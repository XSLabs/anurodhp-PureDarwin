#ifndef _PD_GOP_H
#define _PD_GOP_H

#include <mach/kern_return.h>
#include <stdint.h>

__BEGIN_DECLS

typedef unsigned int mach_port_t;
typedef uint64_t mach_vm_address_t;
typedef uint64_t mach_vm_size_t;

typedef struct PDGOPFramebuffer {
    mach_port_t masterPort;
    mach_port_t service;
    mach_port_t connect;
    mach_vm_address_t address;
    mach_vm_size_t size;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint32_t bpp;
    uint32_t pixelType;
    uint32_t componentMasks[3];
    uint32_t mapOptions;      /* IOConnectMapMemory64 options the VRAM was mapped with */
    uint32_t needsPresent;    /* 1: the service needs PDGOPPresent() after drawing */
} PDGOPFramebuffer;

/*
 * Cache attribute for the VRAM mapping. These are IOKit's kIOMap*Cache
 * values (xnu iokit/IOKit/IOTypes.h: kIOMapCacheShift 8, kIODefaultCache 0,
 * kIOInhibitCache 1, kIOCopybackCache 3, kIOWriteCombineCache 4,
 * kIOPostedWrite 6), which IOUserClient::mapClientMemory64 passes through
 * from user space (kIOMapUserOptionsMask 0xfff includes kIOMapCacheMask).
 * PDGOP_MAP_DEFAULT_CACHE leaves the choice to the kernel's memory
 * descriptor; for a physical range outside the RAM XNU manages (the
 * VideoCore framebuffer on the Pi 3) that is device memory.
 */
#define PDGOP_MAP_DEFAULT_CACHE  0x000u
#define PDGOP_MAP_INHIBIT_CACHE  0x100u   /* VM_WIMG_IO: Device-nGnRnE on arm64 */
#define PDGOP_MAP_COPYBACK_CACHE 0x300u   /* cached; needs explicit cache cleans */
#define PDGOP_MAP_WRITE_COMBINE  0x400u   /* VM_WIMG_WCOMB: Normal non-cacheable */
#define PDGOP_MAP_POSTED_WRITE   0x600u   /* VM_WIMG_POSTED: Device-nGnRE */
#define PDGOP_MAP_CACHE_MASK     0xf00u

kern_return_t PDGOPOpen(PDGOPFramebuffer *fb);
/* PDGOPOpen() with the VRAM mapped using cacheMode (one PDGOP_MAP_* value). */
kern_return_t PDGOPOpenWithCacheMode(PDGOPFramebuffer *fb, uint32_t cacheMode);
void PDGOPClose(PDGOPFramebuffer *fb);
kern_return_t PDGOPPresent(PDGOPFramebuffer *fb,
                           uint32_t x, uint32_t y,
                           uint32_t width, uint32_t height);
const char *PDGOPLastErrorStage(void);

__END_DECLS

#endif /* _PD_GOP_H */
