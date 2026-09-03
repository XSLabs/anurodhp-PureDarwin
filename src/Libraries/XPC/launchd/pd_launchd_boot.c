#include <sys/stat.h>
#include <sys/mount.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "launch.h"
#include "launch_priv.h"
#include "core.h"
#include "log.h"

static void
pd_launchd_boot_mkdir_p(const char *path, mode_t mode)
{
	if (mkdir(path, mode) < 0 && errno != EEXIST) {
		perror(path);
	}
}

static void
pd_launchd_boot_try_mount(const char *src, const char *target, int flags, void *data)
{
	pd_launchd_boot_mkdir_p(target, 0755);
	if (mount(src, target, flags, data) < 0 && errno != EBUSY) {
		fprintf(stderr, "mount %s on %s failed: ", src, target);
		perror("");
	}
}

/* iokit project: root is always mounted read-only by vfs_mountroot()
 * (bsd/vfs/vfs_subr.c hardcodes MNT_RDONLY|MNT_ROOTFS) -- userland PID 1
 * is expected to remount it read-write itself, the real-world equivalent
 * of `mount -uw /`. Without this every write anywhere on "/" fails EROFS.
 * __mac_mount() special-cases the root vnode and forces MNT_UPDATE
 * automatically for path "/", so flags=0 is sufficient, but hfs_mount()
 * (bsd/hfs/hfs_vfsops.c) asserts `data` is non-NULL whenever MNT_UPDATE
 * is set -- a zeroed struct is fine for a plain write-upgrade, those
 * fields only matter for a fresh/wrapper mount. Layout matches the
 * KERNEL-side struct hfs_mount_args (bsd/hfs/hfs_mount.h) field-for-field,
 * not the userspace-visible one -- same struct tools/init_binary/init.c's
 * own remount_root_rw() already uses and documents in more detail. */
struct pd_hfs_mount_args_stub {
	unsigned int hfs_uid;
	unsigned int hfs_gid;
	unsigned short hfs_mask;
	unsigned short _pad;
	unsigned int hfs_encoding;
	int tz_minuteswest;
	int tz_dsttime;
	int flags;
	int journal_tbuffer_size;
	int journal_flags;
	int journal_disable;
};

/* iokit project, DAR-219: report through launchd_syslog(... | LOG_CONSOLE),
 * NOT fprintf(stderr)/perror(). As PID 1 this runs before anything has
 * arranged a useful stderr, so a failing remount used to produce NO output
 * at all -- while its downstream consequence (pd_pid1_prepare_legacy_ipc()'s
 * mkdir("/var/tmp/") failing EROFS a few lines below) DID print, via
 * launchd_syslog|LOG_CONSOLE. A real-hardware boot log therefore showed the
 * symptom with its cause invisible, which is exactly the ambiguity DAR-219
 * had to be investigated to resolve. Log the success case too: silence is
 * not distinguishable from "never ran" (this project has been burned by a
 * diagnostic that shared its subject's failure mode before), and one line
 * per boot is cheap next to re-running a real-hardware test to find out. */
static void
pd_launchd_boot_remount_root_rw(void)
{
	struct pd_hfs_mount_args_stub args;
	memset(&args, 0, sizeof(args));
	if (mount("hfs", "/", 0, &args) < 0) {
		launchd_syslog(LOG_ERR | LOG_CONSOLE,
				"pd_launchd_boot: remount / read-write failed: %s "
				"(root stays read-only; expect EROFS from every write below)",
				strerror(errno));
	} else {
		launchd_syslog(LOG_NOTICE | LOG_CONSOLE,
				"pd_launchd_boot: remounted / read-write");
	}
}

void
pd_launchd_boot(void)
{
	pd_launchd_boot_remount_root_rw();
	pd_launchd_boot_mkdir_p("/dev", 0755);
	pd_launchd_boot_try_mount("devfs", "/dev", 0, NULL);
}
