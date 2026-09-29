/*
 * xf86-input-iohid: Xorg keyboard and pointer input from Apple IOHIDFamily.
 *
 * Replaces xf86-input-puredarwin's reads of /dev/usb_hid_{kbd,mouse} (a
 * PureDarwin-only boot-protocol kext) with the path macOS's own event
 * system uses: IOHIDEventServiceUserClient on each IOHIDEventService
 * (xnu iokit/HID/IOHIDFamily/IOHIDEventServiceUserClient.{h,cpp}):
 *   - IOServiceOpen type 'esuc' (kIOHIDEventServiceUserClientType), which
 *     requires the com.apple.hid.system.user-access-service entitlement, so
 *     the Xorg executable is signed with it;
 *   - selector 0 (kIOHIDEventServiceUserClientOpen) to receive events;
 *   - the event queue mapped with IOConnectMapMemory64 (clientMemoryForType)
 *     and a Mach notification port set on it.
 * Each queue entry is an IOHIDSystemQueueElement header followed by
 * eventCount IOHIDEventData records (IOHIDEvent::readBytes;
 * IOKit/hid/IOHIDEventStructDefs.h for keyboard/pointer/button/scroll).
 *
 * No polling. The notification ports sit in a port set. A helper thread
 * blocks in mach_msg() on that set and writes a byte to a pipe for each
 * notification; the pipe's read end is the device's fd, so Xorg's main loop
 * wakes only when an event arrives. (A kqueue with EVFILT_MACHPORT on the
 * set was tried first: Xorg's ospoll uses poll(), which reported the
 * kqueue fd readable continuously while kevent() returned nothing, and the
 * server spun at ~15-30% CPU. That is xnu's own behaviour: a port-set
 * knote is stay-active, and poll()/select() on the kqueue fd see it as
 * pending (DAR-417, tools/userland_staging/kqueue_poll_smoketest.c). A
 * pipe is an ordinary pollable fd.) The thread stops on a message to a control port in the
 * same set, never by destroying the set under a blocked receive, whose
 * name could be reused before the thread's next mach_msg(). The thread is
 * detached and signals a Mach semaphore as its last access to the driver's
 * state; pthread_join is not exported by this port's libsystem_pthread
 * (DAR-416).
 *
 * Devices. One X input device per kind ("Type" option, "keyboard" or
 * "pointer"). Each opens every IOHIDEventService of that kind:
 * PrimaryUsagePage 1 (Generic Desktop) with usage 6/7 (keyboard, keypad)
 * or 2/1 (mouse, pointer).
 *
 * Keyboard. USB HID usage (page 7) -> Linux evdev keycode, X keycode =
 * evdev + 8, with an evdev XKB keymap. The table is xf86-input-puredarwin's,
 * extended to the navigation keys and keypad. X does its own autorepeat;
 * IOHIDEventService posts no keyboard repeat events (its one use of
 * kIOHIDEventOptionIsRepeat is the multi-axis pointer,
 * IOHIDEventService.cpp:1366).
 *
 * Pointer. Relative motion, buttons and wheel, tracked per service so two
 * mice do not share a button mask. Absolute pointer events
 * (kIOHIDEventOptionIsAbsolute, IOHIDEventTypes.h:604) are ignored.
 *
 * Hot-plug (DAR-418). Services are found through
 * IOServiceAddMatchingNotification(kIOFirstMatchNotification and
 * kIOTerminatedNotification, "IOHIDEventService"), not a one-shot scan. The
 * notification port is a member of the same port set, so the helper thread
 * wakes read_input on a plug or unplug as it does on an event, and
 * read_input drains both iterators (which is also what re-arms them: an
 * IOServiceUserNotification sends only while armed, and it is armed by
 * draining its iterator). The helper thread only receives, and discards the
 * (oversized) notification message, so the callbacks are never dispatched.
 * A device with no service yet is not an error: DEVICE_ON succeeds and the
 * device picks services up as they appear. Each service is identified by its
 * registry entry ID, so a termination closes exactly that service's state.
 *
 * Keyboard LEDs. X's KeybdCtrl.leds (bit 0 Caps, 1 Num, 2 Scroll) is written
 * to each keyboard service with kIOHIDEventServiceUserClientSetElementValue
 * (selector 3: usage page, usage, value; IOHIDEventServiceUserClient.cpp:67,
 * 501-505, which needs the client open), page 8 (LEDs) usages 1-3
 * (IOHIDUsageTables.h:511-513). The last mask is also applied to a keyboard
 * plugged in later.
 *
 * Console. While the keyboard is enabled the driver holds IOHIDSystem's
 * server connection (type kIOHIDServerConnectType, entitlement
 * com.apple.hid.system.server-access) and calls createShmem (selector 0),
 * which sets IOHIDSystem's eventsOpen (xnu iokit/HID/IOHIDSystem/
 * IOHIDSystem.cpp:1386-1389). From then on keyboard events are posted to
 * that event queue instead of the text console's cons_cinput
 * (IOHIDSystem.cpp:2276, 2309-2317). That is how WindowServer takes keys
 * from the console. Closing the connection gives them back: the user
 * client's close() calls evClose (IOHIDUserClient.cpp:121), which clears
 * eventsOpen (IOHIDSystem.cpp:900-921), and that also happens when Xorg
 * dies. This replaces PureDarwin's private IOHIDSystem selector 13, which
 * is not Apple's.
 */
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stddef.h>
#include <time.h>

#include "xf86.h"
#include "xf86_OSproc.h"
#include "xf86str.h"
#include "xf86Module.h"
#include "xf86Xinput.h"
#include "exevents.h"
#include "xkbsrv.h"

#include <mach/mach.h>
#include <mach/semaphore.h>
#include <mach/task.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/IODataQueueClient.h>
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/hid/IOHIDEventTypes.h>
#include <IOKit/hid/IOHIDEventStructDefs.h>

#define kIOHIDEventServiceUserClientType 'esuc'
#define kIOHIDEventServiceUserClientOpen 0
#define kIOHIDEventServiceUserClientSetElementValue 3
#define kHIDPageLEDs                     8
#define kIOHIDServerConnectType          0
#define kIOHIDCreateSharedMemorySelector 0
#define kIOHIDCurrentShmemVersion        4
#define IOHID_MAX_SERVICES               8
/* kQueueSizeMax, IOHIDEventServiceUserClient.cpp:37: no entry is bigger. */
#define IOHID_ENTRY_MAX                  16384

enum IOHIDKind { IOHID_POINTER, IOHID_KEYBOARD };

typedef struct __attribute__((packed, aligned(4))) {
    uint64_t timeStamp;
    uint64_t senderID;
    uint32_t options;
    uint32_t attributeLength;
    uint32_t eventCount;
} IOHIDQueueElementHeader;

/* The fields every IOHIDEventData record starts with (IOHIDEventData.h). */
typedef struct {
    uint32_t size;
    uint32_t type;
    uint32_t options;
} IOHIDEventHeader;
_Static_assert(offsetof(IOHIDKeyboardEventData, options) == offsetof(IOHIDEventHeader, options),
               "IOHIDEventData options offset");

typedef struct {
    io_connect_t       connect;
    IODataQueueMemory *queue;
    mach_port_t        port;
    uint64_t           entryID;        /* registry entry ID of the IOHIDEventService */
    uint32_t           buttons;        /* pointer: last button mask */
    int32_t            fracX, fracY;   /* pointer: IOFixed remainders */
    int32_t            fracSX, fracSY; /* scroll: IOFixed remainders */
} IOHIDService;

typedef struct {
    enum IOHIDKind kind;
    IOHIDService   svc[IOHID_MAX_SERVICES];
    int            nsvc;
    mach_port_t    portSet;
    mach_port_t    ctlPort;        /* in portSet: wakes the thread to stop */
    semaphore_t    threadDone;     /* signalled by the thread as its last act */
    Bool           threadRunning;
    volatile int   stopping;
    int            pipeFd[2];      /* thread -> read_input; [0] is pInfo->fd */
    io_connect_t   hidSystem;      /* keyboard only: console handoff */
    IONotificationPortRef notifyPort; /* hot-plug; its port is in portSet */
    io_iterator_t  matchIter;      /* kIOFirstMatchNotification */
    io_iterator_t  termIter;       /* kIOTerminatedNotification */
    int            leds;           /* keyboard: X's last LED mask */
    Bool           ledsValid;
    uint8_t       *buf;
    unsigned long  nRecords;       /* HID event records decoded, for the close log */
} IOHIDPrivRec, *IOHIDPrivPtr;

/* USB HID keyboard usage -> Linux evdev keycode (X keycode = evdev + 8). */
static const uint16_t usbToEvdev[256] = {
    [0x04] = 30, [0x05] = 48, [0x06] = 46, [0x07] = 32, [0x08] = 18,
    [0x09] = 33, [0x0a] = 34, [0x0b] = 35, [0x0c] = 23, [0x0d] = 36,
    [0x0e] = 37, [0x0f] = 38, [0x10] = 50, [0x11] = 49, [0x12] = 24,
    [0x13] = 25, [0x14] = 16, [0x15] = 19, [0x16] = 31, [0x17] = 20,
    [0x18] = 22, [0x19] = 47, [0x1a] = 17, [0x1b] = 45, [0x1c] = 21,
    [0x1d] = 44,
    [0x1e] = 2, [0x1f] = 3, [0x20] = 4, [0x21] = 5, [0x22] = 6,
    [0x23] = 7, [0x24] = 8, [0x25] = 9, [0x26] = 10, [0x27] = 11,
    [0x28] = 28, [0x29] = 1, [0x2a] = 14, [0x2b] = 15, [0x2c] = 57,
    [0x2d] = 12, [0x2e] = 13, [0x2f] = 26, [0x30] = 27, [0x31] = 43,
    [0x32] = 43, [0x33] = 39, [0x34] = 40, [0x35] = 41, [0x36] = 51,
    [0x37] = 52, [0x38] = 53, [0x39] = 58,
    [0x3a] = 59, [0x3b] = 60, [0x3c] = 61, [0x3d] = 62, [0x3e] = 63,
    [0x3f] = 64, [0x40] = 65, [0x41] = 66, [0x42] = 67, [0x43] = 68,
    [0x44] = 87, [0x45] = 88,
    [0x46] = 99,  /* PrintScreen */ [0x47] = 70,  /* ScrollLock */
    [0x48] = 119, /* Pause */       [0x49] = 110, /* Insert */
    [0x4a] = 102, /* Home */        [0x4b] = 104, /* PageUp */
    [0x4c] = 111, /* Delete */      [0x4d] = 107, /* End */
    [0x4e] = 109, /* PageDown */
    [0x4f] = 106, [0x50] = 105, [0x51] = 108, [0x52] = 103,
    [0x53] = 69,  /* NumLock */     [0x54] = 98,  /* KP / */
    [0x55] = 55,  /* KP * */        [0x56] = 74,  /* KP - */
    [0x57] = 78,  /* KP + */        [0x58] = 96,  /* KP Enter */
    [0x59] = 79, [0x5a] = 80, [0x5b] = 81, [0x5c] = 75, [0x5d] = 76,
    [0x5e] = 77, [0x5f] = 71, [0x60] = 72, [0x61] = 73, [0x62] = 82,
    [0x63] = 83,  /* KP . */        [0x64] = 86,  /* 102nd key */
    [0x65] = 127, /* Menu */
    [0xe0] = 29, [0xe1] = 42, [0xe2] = 56, [0xe3] = 125,
    [0xe4] = 97, [0xe5] = 54, [0xe6] = 100, [0xe7] = 126,
};

static int
IOHIDIntProp(io_service_t s, const char *key)
{
    CFStringRef k = CFStringCreateWithCString(NULL, key, kCFStringEncodingUTF8);
    CFTypeRef v = k ? IORegistryEntryCreateCFProperty(s, k, kCFAllocatorDefault, 0) : NULL;
    int out = -1;
    if (v && CFGetTypeID(v) == CFNumberGetTypeID())
        CFNumberGetValue((CFNumberRef)v, kCFNumberIntType, &out);
    if (v) CFRelease(v);
    if (k) CFRelease(k);
    return out;
}

static Bool
IOHIDWantService(enum IOHIDKind kind, int page, int usage)
{
    if (page != 1)
        return FALSE;
    if (kind == IOHID_KEYBOARD)
        return usage == 6 || usage == 7;
    return usage == 2 || usage == 1;
}

/* IOFixed -> whole device units, carrying the fractional part forward. */
static int
IOHIDTakeUnits(IOFixed v, int32_t *frac)
{
    int32_t total = *frac + v;
    int whole = total / 65536;
    *frac = total - whole * 65536;
    return whole;
}

static void
IOHIDPostButton(InputInfoPtr pInfo, int button, int down)
{
    xf86PostButtonEvent(pInfo->dev, 0, button, down, 0, 0);
}

static void
IOHIDDecode(InputInfoPtr pInfo, IOHIDService *svc, const uint8_t *buf, uint32_t len)
{
    IOHIDPrivPtr priv = pInfo->private;
    const IOHIDQueueElementHeader *h = (const IOHIDQueueElementHeader *)buf;
    uint64_t off;

    if (len < sizeof(*h))
        return;
    off = (uint64_t)sizeof(*h) + h->attributeLength;
    for (uint32_t i = 0; i < h->eventCount && off + 16 <= len; i++) {
        const IOHIDKeyboardEventData *base = (const IOHIDKeyboardEventData *)(buf + off);
        const IOHIDEventHeader *hdr = (const IOHIDEventHeader *)(buf + off);
        uint32_t size = base->size;
        if (size < 16 || off + size > len)
            break;
        priv->nRecords++;
        switch (base->type) {
        case kIOHIDEventTypeKeyboard:
            if (priv->kind == IOHID_KEYBOARD && size >= sizeof(IOHIDKeyboardEventData) &&
                base->usagePage == 7 && base->usage < 256) {
                uint16_t evdev = usbToEvdev[base->usage];
                if (evdev)
                    xf86PostKeyboardEvent(pInfo->dev, evdev + 8, base->down ? TRUE : FALSE);
            }
            break;
        case kIOHIDEventTypePointer:
            if (priv->kind == IOHID_POINTER && size >= sizeof(IOHIDPointerEventData) &&
                !(hdr->options & kIOHIDEventOptionIsAbsolute)) {
                const IOHIDPointerEventData *p = (const IOHIDPointerEventData *)base;
                int dx = IOHIDTakeUnits(p->position.x, &svc->fracX);
                int dy = IOHIDTakeUnits(p->position.y, &svc->fracY);
                if (dx || dy)
                    xf86PostMotionEvent(pInfo->dev, 0, 0, 2, dx, dy);
            }
            break;
        case kIOHIDEventTypeButton:
            if (priv->kind == IOHID_POINTER && size >= sizeof(IOHIDButtonEventData)) {
                /* HID button bit 0 left, 1 right, 2 middle -> X 1, 3, 2;
                 * further buttons -> X 8, 9, ... (4-7 are the wheel). */
                const IOHIDButtonEventData *b = (const IOHIDButtonEventData *)base;
                uint32_t changed = b->mask ^ svc->buttons;
                for (int bit = 0; bit < 16; bit++) {
                    if (!(changed & (1u << bit)))
                        continue;
                    int xb = bit == 0 ? 1 : bit == 1 ? 3 : bit == 2 ? 2 : 8 + (bit - 3);
                    IOHIDPostButton(pInfo, xb, (b->mask >> bit) & 1);
                }
                svc->buttons = b->mask;
            }
            break;
        case kIOHIDEventTypeScroll:
            if (priv->kind == IOHID_POINTER && size >= sizeof(IOHIDScrollEventData)) {
                /* One click per whole unit, fractions carried forward.
                 * +y is up (X 4), -y down (5): the wheel is not negated
                 * (IOHIDEventDriver.cpp:3779), and a USB wheel click up
                 * reads y +1.0 (QEMU usb-mouse, hid_event_smoketest).
                 * +x is LEFT (X 6), -x right (7): AC Pan is negated
                 * (IOHIDEventDriver.cpp:3791), the macOS convention. */
                const IOHIDScrollEventData *s = (const IOHIDScrollEventData *)base;
                int ny = IOHIDTakeUnits(s->position.y, &svc->fracSY);
                int nx = IOHIDTakeUnits(s->position.x, &svc->fracSX);
                for (int k = 0; k < (ny < 0 ? -ny : ny); k++) {
                    IOHIDPostButton(pInfo, ny > 0 ? 4 : 5, 1);
                    IOHIDPostButton(pInfo, ny > 0 ? 4 : 5, 0);
                }
                for (int k = 0; k < (nx < 0 ? -nx : nx); k++) {
                    IOHIDPostButton(pInfo, nx > 0 ? 6 : 7, 1);
                    IOHIDPostButton(pInfo, nx > 0 ? 6 : 7, 0);
                }
            }
            break;
        default:
            break;
        }
        off += size;
    }
}

/* Helper thread: one pipe byte per notification message. */
static void *
IOHIDNotifyThread(void *arg)
{
    IOHIDPrivPtr priv = arg;
    sigset_t all;

    /* Xorg's signals (SIGIO, SIGCHLD, SIGALRM, ...) belong to its main
     * thread, as its own input thread arranges (os/inputthread.c). */
    sigfillset(&all);
    pthread_sigmask(SIG_BLOCK, &all, NULL);
    for (;;) {
        struct { mach_msg_header_t h; mach_msg_trailer_t t; } msg;
        kern_return_t kr = mach_msg(&msg.h, MACH_RCV_MSG, 0, sizeof(msg), priv->portSet,
                                    MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        if (priv->stopping)
            break;
        if (kr == KERN_SUCCESS || kr == MACH_RCV_TOO_LARGE) {
            char b = 1;
            /* Non-blocking: a full pipe already means "events pending". */
            (void)write(priv->pipeFd[1], &b, 1);
        } else if (kr != MACH_RCV_INTERRUPTED) {
            LogMessageVerbSigSafe(X_ERROR, 0, "iohid: notification receive failed (0x%x); "
                                  "input from this device stops\n", kr);
            break;
        }
    }
    semaphore_signal(priv->threadDone);   /* last touch of priv */
    return NULL;
}

static void
IOHIDSetLEDs(IOHIDService *e, int leds)
{
    /* X LED bit n -> HID LED usage: 0 Caps (2), 1 Num (1), 2 Scroll (3). */
    static const uint64_t usage[3] = { 2, 1, 3 };

    for (int bit = 0; bit < 3; bit++) {
        uint64_t in[3] = { kHIDPageLEDs, usage[bit], (leds >> bit) & 1 };
        /* Best effort: a keyboard without that LED refuses, which is fine. */
        (void)IOConnectCallScalarMethod(e->connect, kIOHIDEventServiceUserClientSetElementValue,
                                        in, 3, NULL, NULL);
    }
}

static void
IOHIDCloseService(IOHIDService *e)
{
    if (e->queue)
        IOConnectUnmapMemory64(e->connect, 0, mach_task_self(), (mach_vm_address_t)(uintptr_t)e->queue);
    if (e->connect)
        IOServiceClose(e->connect);
    if (e->port)
        mach_port_mod_refs(mach_task_self(), e->port, MACH_PORT_RIGHT_RECEIVE, -1);
    memset(e, 0, sizeof(*e));
}

/* Open one wanted service into the next free slot. */
static void
IOHIDOpenService(InputInfoPtr pInfo, io_service_t s)
{
    IOHIDPrivPtr priv = pInfo->private;
    int page = IOHIDIntProp(s, "PrimaryUsagePage"), usage = IOHIDIntProp(s, "PrimaryUsage");
    uint64_t id = 0;
    IOHIDService *e;
    mach_vm_address_t addr = 0;
    mach_vm_size_t size = 0;
    uint64_t options = 0;
    kern_return_t kr;

    if (!IOHIDWantService(priv->kind, page, usage))
        return;
    if (priv->nsvc >= IOHID_MAX_SERVICES) {
        xf86IDrvMsg(pInfo, X_WARNING, "more than %d IOHIDEventServices; ignoring one\n",
                    IOHID_MAX_SERVICES);
        return;
    }
    if (IORegistryEntryGetRegistryEntryID(s, &id) != KERN_SUCCESS)
        id = 0;
    for (int i = 0; id && i < priv->nsvc; i++)
        if (priv->svc[i].entryID == id)
            return;                     /* already open */

    e = &priv->svc[priv->nsvc];
    /* Fresh state: a button held across DEVICE_OFF was released
     * by X, so a stale mask would swallow the next press. */
    memset(e, 0, sizeof(*e));
    e->entryID = id;
    kr = IOServiceOpen(s, mach_task_self(), kIOHIDEventServiceUserClientType, &e->connect);
    if (!kr)
        kr = IOConnectCallScalarMethod(e->connect, kIOHIDEventServiceUserClientOpen, &options, 1, NULL, NULL);
    if (!kr)
        kr = IOConnectMapMemory64(e->connect, 0, mach_task_self(), &addr, &size, kIOMapAnywhere);
    if (!kr) {
        e->port = IODataQueueAllocateNotificationPort();
        kr = e->port ? IOConnectSetNotificationPort(e->connect, 0, e->port, 0) : KERN_FAILURE;
    }
    if (!kr)
        kr = mach_port_insert_member(mach_task_self(), e->port, priv->portSet);
    if (kr) {
        xf86IDrvMsg(pInfo, X_WARNING, "IOHIDEventService (usage %d:%d): open failed 0x%x\n",
                    page, usage, kr);
        if (addr)
            IOConnectUnmapMemory64(e->connect, 0, mach_task_self(), addr);
        e->queue = NULL;
        IOHIDCloseService(e);
        return;
    }
    e->queue = (IODataQueueMemory *)(uintptr_t)addr;
    xf86IDrvMsg(pInfo, X_INFO, "IOHIDEventService (usage %d:%d) opened\n", page, usage);
    priv->nsvc++;
    if (priv->kind == IOHID_KEYBOARD && priv->ledsValid)
        IOHIDSetLEDs(e, priv->leds);
}

/* Drain the first-match iterator (which re-arms it): open what it returns. */
static void
IOHIDScanMatched(InputInfoPtr pInfo)
{
    IOHIDPrivPtr priv = pInfo->private;
    io_service_t s;

    if (!priv->matchIter)
        return;
    while ((s = IOIteratorNext(priv->matchIter)) != IO_OBJECT_NULL) {
        IOHIDOpenService(pInfo, s);
        IOObjectRelease(s);
    }
}

/* Drain the terminated iterator: close the state of each service that went. */
static void
IOHIDScanTerminated(InputInfoPtr pInfo)
{
    IOHIDPrivPtr priv = pInfo->private;
    io_service_t s;

    if (!priv->termIter)
        return;
    while ((s = IOIteratorNext(priv->termIter)) != IO_OBJECT_NULL) {
        uint64_t id = 0;
        if (IORegistryEntryGetRegistryEntryID(s, &id) == KERN_SUCCESS) {
            for (int i = 0; i < priv->nsvc; i++) {
                if (priv->svc[i].entryID != id)
                    continue;
                xf86IDrvMsg(pInfo, X_INFO, "IOHIDEventService removed\n");
                IOHIDCloseService(&priv->svc[i]);
                for (int j = i + 1; j < priv->nsvc; j++)
                    priv->svc[j - 1] = priv->svc[j];
                memset(&priv->svc[--priv->nsvc], 0, sizeof(IOHIDService));
                break;
            }
        }
        IOObjectRelease(s);
    }
}

static void
IOHIDNotifyNoop(void *refcon, io_iterator_t iterator)
{
    (void)refcon;
    (void)iterator;
}

/* Returns 0 on failure. Finding no service yet is success. */
static int
IOHIDOpenServices(InputInfoPtr pInfo)
{
    IOHIDPrivPtr priv = pInfo->private;
    mach_port_t np;

    if (mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_PORT_SET, &priv->portSet) != KERN_SUCCESS)
        return 0;
    if (mach_port_allocate(mach_task_self(), MACH_PORT_RIGHT_RECEIVE, &priv->ctlPort) != KERN_SUCCESS ||
        mach_port_insert_right(mach_task_self(), priv->ctlPort, priv->ctlPort, MACH_MSG_TYPE_MAKE_SEND) != KERN_SUCCESS ||
        mach_port_insert_member(mach_task_self(), priv->ctlPort, priv->portSet) != KERN_SUCCESS)
        return 0;
    priv->notifyPort = IONotificationPortCreate(kIOMasterPortDefault);
    if (!priv->notifyPort)
        return 0;
    np = IONotificationPortGetMachPort(priv->notifyPort);
    if (mach_port_insert_member(mach_task_self(), np, priv->portSet) != KERN_SUCCESS)
        return 0;
    if (IOServiceAddMatchingNotification(priv->notifyPort, kIOTerminatedNotification,
                                         IOServiceMatching("IOHIDEventService"),
                                         IOHIDNotifyNoop, NULL, &priv->termIter) != KERN_SUCCESS ||
        IOServiceAddMatchingNotification(priv->notifyPort, kIOFirstMatchNotification,
                                         IOServiceMatching("IOHIDEventService"),
                                         IOHIDNotifyNoop, NULL, &priv->matchIter) != KERN_SUCCESS)
        return 0;
    /* Arm the terminated iterator (it starts empty), then take what exists. */
    IOHIDScanTerminated(pInfo);
    IOHIDScanMatched(pInfo);
    return 1;
}

static void
IOHIDReadInput(InputInfoPtr pInfo)
{
    IOHIDPrivPtr priv = pInfo->private;
    char drain[64];

    while (read(pInfo->fd, drain, sizeof(drain)) > 0)
        ;
    /* Every wake may be a plug or unplug rather than events. Unplugs first,
     * so a replug of the same slot is not seen as a duplicate. */
    IOHIDScanTerminated(pInfo);
    IOHIDScanMatched(pInfo);
    for (int i = 0; i < priv->nsvc; i++) {
        IOHIDService *s = &priv->svc[i];
        uint32_t len = IOHID_ENTRY_MAX;
        IOReturn r;
        while ((r = IODataQueueDequeue(s->queue, priv->buf, &len)) != kIOReturnUnderrun) {
            if (r == kIOReturnSuccess) {
                IOHIDDecode(pInfo, s, priv->buf, len);
            } else {
                /* An entry that does not fit is skipped, never left at the
                 * head where it would wedge the queue. */
                uint32_t skip = 0;
                if (IODataQueueDequeue(s->queue, NULL, &skip) != kIOReturnSuccess)
                    break;
            }
            len = IOHID_ENTRY_MAX;
        }
    }
}

static Bool
IOHIDStartThread(IOHIDPrivPtr priv)
{
    pthread_attr_t attr;
    pthread_t thread;
    int err;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    err = pthread_create(&thread, &attr, IOHIDNotifyThread, priv);
    pthread_attr_destroy(&attr);
    if (err) {
        errno = err;
        return FALSE;
    }
    return TRUE;
}

static void
IOHIDStopThread(IOHIDPrivPtr priv)
{
    if (priv->threadRunning) {
        mach_msg_header_t h = { 0 };
        priv->stopping = 1;
        h.msgh_bits = MACH_MSGH_BITS(MACH_MSG_TYPE_COPY_SEND, 0);
        h.msgh_size = sizeof(h);
        h.msgh_remote_port = priv->ctlPort;
        mach_msg(&h, MACH_SEND_MSG, sizeof(h), 0, MACH_PORT_NULL, MACH_MSG_TIMEOUT_NONE, MACH_PORT_NULL);
        while (semaphore_wait(priv->threadDone) == KERN_ABORTED)
            ;
        priv->threadRunning = FALSE;
    }
    if (priv->threadDone) {
        semaphore_destroy(mach_task_self(), priv->threadDone);
        priv->threadDone = MACH_PORT_NULL;
    }
    priv->stopping = 0;
    for (int i = 0; i < 2; i++) {
        if (priv->pipeFd[i] >= 0)
            close(priv->pipeFd[i]);
        priv->pipeFd[i] = -1;
    }
}

static void
IOHIDCloseServices(IOHIDPrivPtr priv)
{
    IOHIDStopThread(priv);
    for (int i = 0; i < priv->nsvc; i++)
        IOHIDCloseService(&priv->svc[i]);
    priv->nsvc = 0;
    if (priv->matchIter) {
        IOObjectRelease(priv->matchIter);
        priv->matchIter = IO_OBJECT_NULL;
    }
    if (priv->termIter) {
        IOObjectRelease(priv->termIter);
        priv->termIter = IO_OBJECT_NULL;
    }
    if (priv->notifyPort) {
        IONotificationPortDestroy(priv->notifyPort);
        priv->notifyPort = NULL;
    }
    if (priv->ctlPort) {
        mach_port_mod_refs(mach_task_self(), priv->ctlPort, MACH_PORT_RIGHT_RECEIVE, -1);
        mach_port_deallocate(mach_task_self(), priv->ctlPort);
        priv->ctlPort = MACH_PORT_NULL;
    }
    if (priv->portSet) {
        mach_port_mod_refs(mach_task_self(), priv->portSet, MACH_PORT_RIGHT_PORT_SET, -1);
        priv->portSet = MACH_PORT_NULL;
    }
    if (priv->hidSystem) {
        IOServiceClose(priv->hidSystem);   /* evClose: keys go back to the console */
        priv->hidSystem = IO_OBJECT_NULL;
    }
}

/* Keyboard: take keys from the text console the way WindowServer does. */
static void
IOHIDAcquireConsole(InputInfoPtr pInfo)
{
    IOHIDPrivPtr priv = pInfo->private;
    io_service_t sys = IOServiceGetMatchingService(kIOMasterPortDefault, IOServiceMatching("IOHIDSystem"));
    kern_return_t kr;
    uint64_t version = kIOHIDCurrentShmemVersion;

    if (sys == IO_OBJECT_NULL)
        return;
    /* evOpen refuses a second server (kIOReturnBusy, IOHIDSystem.cpp:873-876)
     * until the previous connection's evClose, which runs from the user
     * client's stop() after an asynchronous terminate()
     * (IOHIDUserClient.cpp:104-106, 121). A server regeneration can get
     * here first, so wait up to a second. */
    for (int tries = 0; ; tries++) {
        kr = IOServiceOpen(sys, mach_task_self(), kIOHIDServerConnectType, &priv->hidSystem);
        if (kr != kIOReturnBusy || tries == 20)
            break;
        struct timespec ts = { 0, 50 * 1000 * 1000 };
        nanosleep(&ts, NULL);
    }
    IOObjectRelease(sys);
    if (kr) {
        xf86IDrvMsg(pInfo, X_ERROR, "IOHIDSystem server connect failed (0x%x): "
                    "keys typed in X ALSO reach the text console\n", kr);
        priv->hidSystem = IO_OBJECT_NULL;
        return;
    }
    kr = IOConnectCallScalarMethod(priv->hidSystem, kIOHIDCreateSharedMemorySelector, &version, 1, NULL, NULL);
    if (kr)
        xf86IDrvMsg(pInfo, X_ERROR, "IOHIDSystem createShmem failed (0x%x): "
                    "keys typed in X ALSO reach the text console\n", kr);
    else
        xf86IDrvMsg(pInfo, X_INFO, "IOHIDSystem events open: keys no longer reach the text console\n");
}

static void IOHIDPtrCtrl(DeviceIntPtr dev, PtrCtrl *ctrl) { (void)dev; (void)ctrl; }
static void
IOHIDKbdCtrl(DeviceIntPtr dev, KeybdCtrl *ctrl)
{
    InputInfoPtr pInfo = dev->public.devicePrivate;
    IOHIDPrivPtr priv = pInfo ? pInfo->private : NULL;

    if (!priv || priv->kind != IOHID_KEYBOARD)
        return;
    priv->leds = ctrl->leds;
    priv->ledsValid = TRUE;
    for (int i = 0; i < priv->nsvc; i++)
        IOHIDSetLEDs(&priv->svc[i], priv->leds);
}

static int
IOHIDDeviceControl(DeviceIntPtr dev, int what)
{
    InputInfoPtr pInfo = dev->public.devicePrivate;
    IOHIDPrivPtr priv = pInfo->private;

    switch (what) {
    case DEVICE_INIT:
        if (priv->kind == IOHID_POINTER) {
            CARD8 map[16];
            Atom btnLabels[15] = { 0 };
            Atom axisLabels[2] = { 0 };
            for (int i = 0; i < 16; i++)
                map[i] = i;
            if (!InitPointerDeviceStruct((DevicePtr)dev, map, 15, btnLabels, IOHIDPtrCtrl,
                                         GetMotionHistorySize(), 2, axisLabels))
                return BadAlloc;
            xf86InitValuatorAxisStruct(dev, 0, axisLabels[0], -1, -1, 1, 0, 1, Relative);
            xf86InitValuatorAxisStruct(dev, 1, axisLabels[1], -1, -1, 1, 0, 1, Relative);
        } else {
            XkbRMLVOSet rmlvo = { 0 };
            rmlvo.rules = "evdev";
            rmlvo.model = "pc105";
            rmlvo.layout = "us";
            if (!InitKeyboardDeviceStruct(dev, &rmlvo, NULL, IOHIDKbdCtrl)) {
                xf86IDrvMsg(pInfo, X_ERROR, "keymap (evdev/pc105/us) could not be built\n");
                return BadValue;
            }
        }
        break;

    case DEVICE_ON:
        if (pInfo->fd < 0) {
            if (IOHIDOpenServices(pInfo) == 0) {
                xf86IDrvMsg(pInfo, X_ERROR, "IOHIDEventService notifications could not be set up\n");
                IOHIDCloseServices(priv);
                return BadRequest;
            }
            if (priv->nsvc == 0)
                xf86IDrvMsg(pInfo, X_INFO, "no %s IOHIDEventService yet; waiting for one\n",
                            priv->kind == IOHID_KEYBOARD ? "keyboard" : "pointer");
            if (pipe(priv->pipeFd) < 0 ||
                fcntl(priv->pipeFd[0], F_SETFL, O_NONBLOCK) < 0 ||
                fcntl(priv->pipeFd[1], F_SETFL, O_NONBLOCK) < 0 ||
                fcntl(priv->pipeFd[0], F_SETFD, FD_CLOEXEC) < 0 ||
                fcntl(priv->pipeFd[1], F_SETFD, FD_CLOEXEC) < 0 ||
                semaphore_create(mach_task_self(), &priv->threadDone, SYNC_POLICY_FIFO, 0) != KERN_SUCCESS ||
                !IOHIDStartThread(priv)) {
                xf86IDrvMsg(pInfo, X_ERROR, "notification pipe/thread failed: %s\n", strerror(errno));
                IOHIDCloseServices(priv);
                return BadRequest;
            }
            priv->threadRunning = TRUE;
            pInfo->fd = priv->pipeFd[0];
            if (priv->kind == IOHID_KEYBOARD)
                IOHIDAcquireConsole(pInfo);
        }
        xf86AddEnabledDevice(pInfo);
        dev->public.on = TRUE;
        break;

    case DEVICE_OFF:
    case DEVICE_CLOSE:
        if (pInfo->fd >= 0) {
            xf86IDrvMsg(pInfo, X_INFO, "%lu HID event record(s) decoded\n", priv->nRecords);
            xf86RemoveEnabledDevice(pInfo);
            pInfo->fd = -1;             /* pipeFd[0], closed below */
        }
        IOHIDCloseServices(priv);
        dev->public.on = FALSE;
        break;
    }
    return Success;
}

static int
IOHIDPreInit(InputDriverPtr drv, InputInfoPtr pInfo, int flags)
{
    IOHIDPrivPtr priv;
    char *type;

    (void)drv;
    (void)flags;
    priv = calloc(1, sizeof(IOHIDPrivRec));
    if (!priv)
        return BadAlloc;
    priv->buf = malloc(IOHID_ENTRY_MAX);
    if (!priv->buf) {
        free(priv);
        return BadAlloc;
    }
    priv->pipeFd[0] = priv->pipeFd[1] = -1;
    pInfo->private = priv;

    type = xf86SetStrOption(pInfo->options, "Type", "pointer");
    if (type && !strcasecmp(type, "keyboard")) {
        priv->kind = IOHID_KEYBOARD;
        pInfo->type_name = XI_KEYBOARD;
    } else {
        priv->kind = IOHID_POINTER;
        pInfo->type_name = XI_MOUSE;
    }
    free(type);
    pInfo->device_control = IOHIDDeviceControl;
    pInfo->read_input = IOHIDReadInput;
    pInfo->fd = -1;
    xf86IDrvMsg(pInfo, X_INFO, "IOHIDFamily %s\n", priv->kind == IOHID_KEYBOARD ? "keyboard" : "pointer");
    return Success;
}

static void
IOHIDUnInit(InputDriverPtr drv, InputInfoPtr pInfo, int flags)
{
    IOHIDPrivPtr priv = pInfo ? pInfo->private : NULL;

    (void)drv;
    if (priv) {
        IOHIDCloseServices(priv);
        free(priv->buf);
        free(priv);
        pInfo->private = NULL;
    }
    xf86DeleteInput(pInfo, flags);
}

static InputDriverRec IOHID = {
    .driverVersion = 1,
    .driverName    = "iohid",
    .PreInit       = IOHIDPreInit,
    .UnInit        = IOHIDUnInit,
    .module        = NULL,
    .default_options = NULL,
};

static void *
IOHIDPlug(void *module, void *options, int *errmaj, int *errmin)
{
    (void)options;
    (void)errmaj;
    (void)errmin;
    xf86AddInputDriver(&IOHID, module, 0);
    return module;
}

static XF86ModuleVersionInfo IOHIDVersionRec = {
    "iohid",
    MODULEVENDORSTRING,
    MODINFOSTRING1,
    MODINFOSTRING2,
    XORG_VERSION_CURRENT,
    1, 0, 0,
    ABI_CLASS_XINPUT,
    ABI_XINPUT_VERSION,
    MOD_CLASS_XINPUT,
    { 0, 0, 0, 0 }
};

_X_EXPORT XF86ModuleData iohidModuleData = {
    &IOHIDVersionRec,
    IOHIDPlug,
    NULL
};
