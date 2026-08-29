/*
 * Copyright (c) 2005 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_APACHE_LICENSE_HEADER_START@
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @APPLE_APACHE_LICENSE_HEADER_END@
 */

#include "config.h"
#include "launchd.h"

#include <sys/types.h>
#include <sys/queue.h>
#include <sys/event.h>
#include <sys/stat.h>
#include <sys/ucred.h>
#include <sys/fcntl.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <sys/sysctl.h>
#include <sys/sockio.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/kern_event.h>
#include <sys/reboot.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/in_var.h>
#include <netinet6/nd6.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <paths.h>
#include <pwd.h>
#include <grp.h>
#include <ttyent.h>
#include <dlfcn.h>
#include <dirent.h>
#include <string.h>
#include <setjmp.h>
#include <spawn.h>
#include <sched.h>
#include <pthread.h>
#include <util.h>
#include <os/assumes.h>

#if HAVE_LIBAUDITD
/* PureDarwin: bsm/auditd_lib.h doesn't exist in this (or any modern) SDK -
 * its only job here was declaring _audit_session_self(), a private
 * double-underscore-mangled forwarder to the real "audit_session_self" Mach
 * syscall */
#include <mach/mach_types.h>
extern mach_port_name_t audit_session_self(void);
static mach_port_name_t
_audit_session_self(void)
{
	return audit_session_self();
}
//#include <bsm/audit_session.h> // isn't in openbsm, can we do without it?
#endif

#include <bootstrap.h>
#include <vproc.h>
#include <vproc_priv.h>
#include "vproc_internal.h"
#include <launch.h>
#include <launch_internal.h>

#include "runtime.h"
#include "core.h"
#include "ipc.h"

#define LAUNCHD_CONF ".launchd.conf"

extern char **environ;

static void pfsystem_callback(void *, struct kevent *);

static kq_callback kqpfsystem_callback = pfsystem_callback;

static void pid1_magic_init(void);

static void testfd_or_openfd(int fd, const char *path, int flags);
static bool get_network_state(void);
static void monitor_networking_state(void);
static void fatal_signal_handler(int sig, siginfo_t *si, void *uap);
static void handle_pid1_crashes_separately(void);
static void do_pid1_crash_diagnosis_mode(const char *msg);
static int basic_fork(void);
static bool do_pid1_crash_diagnosis_mode2(const char *msg);
static void pd_pid1_prepare_legacy_ipc(void);

static void *update_thread(void *nothing);

static void *crash_addr;
static pid_t crash_pid;

char *_launchd_database_dir;
char *_launchd_log_dir;

bool launchd_shutting_down;
bool network_up;
uid_t launchd_uid;
FILE *launchd_console = NULL;
int32_t launchd_sync_frequency = 30;


/* Real, locally-defined (this project's own static build, see
 * tools/userland_staging/pthread_main_thread_bootstrap.c) -- no header
 * exposes this prototype to launchd's own real PureDarwin source, so
 * declared here directly. Used below in main(), bug #6 investigation. */
extern int pthread_main_thread_bootstrap(void);

/* TEMP DIAGNOSTIC: bisecting a silent exit(1) with zero console output
 * anywhere before it -- first round (this project's normal open()/
 * write()/strlen()) produced NO output at all, not even "main() entry"
 * at the very top of main(), which means either main() itself is never
 * reached (something before it, e.g. a real dyld/libc++/ICU static
 * initializer, calls exit(1) first) or this project's own open()/write()
 * wrappers have a bug. This second round uses RAW syscalls (matching
 * crt0_smoketest.c's own proven-working pattern), bypassing libc's
 * open()/write() entirely, to isolate which. Remove once root-caused. */
static long
iokit_diag_syscall3(long num, long a0, long a1, long a2)
{
	register long x0 asm("x0") = a0;
	register long x1 asm("x1") = a1;
	register long x2 asm("x2") = a2;
	register long x16 asm("x16") = num;
	asm volatile("svc #0x80" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x16) : "memory", "cc");
	return x0;
}

static void
iokit_diag(const char *msg)
{
	long len = 0;
	while (msg[len] != '\0') {
		len++;
	}
	long fd = iokit_diag_syscall3(5 /* SYS_open */, (long)_PATH_CONSOLE, O_WRONLY | O_NOCTTY, 0);
	if (fd < 0) {
		return;
	}
	iokit_diag_syscall3(4 /* SYS_write */, fd, (long)msg, len);
	iokit_diag_syscall3(6 /* SYS_close */, fd, 0, 0);
}

__attribute__((constructor))
static void
iokit_diag_ctor(void)
{
	iokit_diag("DIAG: __mod_init_func constructor reached (before main())\n");
}

int
main(int argc, char *const *argv)
{
	bool sflag = false;
	int ch;

	iokit_diag("DIAG: main() entry\n");

	/* This needs to be cleaned up. Currently, we risk tripping assumes() macros
	 * before we've properly set things like launchd's log database paths, the
	 * global launchd label for syslog messages and the like. Luckily, these are
	 * operations that will probably never fail, like test_of_openfd(), the
	 * stuff in launchd_runtime_init() and the stuff in
	 * handle_pid1_crashes_separately().
	 */
	testfd_or_openfd(STDIN_FILENO, _PATH_DEVNULL, O_RDONLY);
	testfd_or_openfd(STDOUT_FILENO, _PATH_DEVNULL, O_WRONLY);
	testfd_or_openfd(STDERR_FILENO, _PATH_DEVNULL, O_WRONLY);
	iokit_diag("DIAG: past testfd_or_openfd\n");

	if (launchd_use_gmalloc) {
		if (!getenv("DYLD_INSERT_LIBRARIES")) {
			setenv("DYLD_INSERT_LIBRARIES", "/usr/lib/libgmalloc.dylib", 1);
			setenv("MALLOC_STRICT_SIZE", "1", 1);
			execv(argv[0], argv);
		} else {
			unsetenv("DYLD_INSERT_LIBRARIES");
			unsetenv("MALLOC_STRICT_SIZE");
		}
	} else if (launchd_malloc_log_stacks) {
		if (!getenv("MallocStackLogging")) {
			setenv("MallocStackLogging", "1", 1);
			execv(argv[0], argv);
		} else {
			unsetenv("MallocStackLogging");
		}
	}

	while ((ch = getopt(argc, argv, "s")) != -1) {
		switch (ch) {
		case 's': sflag = true; break;	/* single user */
		case '?': /* we should do something with the global optopt variable here */
		default:
			fprintf(stderr, "%s: ignoring unknown arguments\n", getprogname());
			break;
		}
	}

	if (getpid() != 1 && getppid() != 1) {
		iokit_diag("DIAG: failing getpid/getppid check, exiting\n");
		fprintf(stderr, "%s: This program is not meant to be run directly.\n", getprogname());
		exit(EXIT_FAILURE);
	}
	iokit_diag("DIAG: passed getpid/getppid check\n");

	/* REAL BUG under investigation (bug #6, session 2): launchd.macho's
	 * own statically-linked pthread machinery (runtime.c's real
	 * pthread_create() call inside launchd_runtime_init(), below) has
	 * its OWN separate, local copy of _main_thread/__pthread_head/
	 * pthread_main_thread_bootstrap (confirmed via `nm launchd.macho` --
	 * all real, locally-defined `T`/`S`/`d` symbols, not `U`) -- a
	 * SEPARATE image's worth of pthread state from libsystem_pthread.dylib's
	 * own copy, which libsystem_dyld_initializer_compat.c's constructor
	 * already bootstraps via a cross-dylib call. TPIDRRO_EL0 is a single,
	 * real, process-global CPU register though -- so after that
	 * constructor runs, it points at libsystem_pthread.dylib's own
	 * &_main_thread.tsd[0], not launchd's own local copy's. Any code in
	 * launchd's own local pthread copy that computes pthread_self()/
	 * errno from that same register would get a valid-looking pointer
	 * into the WRONG image's struct -- a real cross-image confusion,
	 * exactly the shape of the new second-thread SIGSEGV this fix is
	 * testing. This is genuinely the FIRST call against THIS copy's
	 * _main_thread/__pthread_head (confirmed via nm -- nothing has
	 * touched them yet), not a double-bootstrap of anything already
	 * initialized once. */
	pthread_main_thread_bootstrap();
	iokit_diag("DIAG: past launchd-local pthread_main_thread_bootstrap\n");

	launchd_runtime_init();
	iokit_diag("DIAG: returned from launchd_runtime_init\n");

	if (NULL == getenv("PATH")) {
		setenv("PATH", _PATH_STDPATH, 1);
	}
	iokit_diag("DIAG: past PATH setenv\n");

	if (pid1_magic) {
		iokit_diag("DIAG: about to call pid1_magic_init\n");
		pid1_magic_init();
		iokit_diag("DIAG: returned from pid1_magic_init\n");

		int cfd = -1;
		if ((cfd = open(_PATH_CONSOLE, O_WRONLY | O_NOCTTY)) != -1) {
			_fd(cfd);
			if (!(launchd_console = fdopen(cfd, "w"))) {
				(void)close(cfd);
			}
		}

		char *extra = "";
		if (launchd_osinstaller) {
			extra = " in the OS Installer";
		} else if (sflag) {
			extra = " in single-user mode";
		}

		launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** launchd[1] has started up%s. ***", extra);
		if (launchd_use_gmalloc) {
			launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** Using libgmalloc. ***");
		}

		if (launchd_verbose_boot) {
			launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** Verbose boot, will log to /dev/console. ***");
		}

		if (launchd_shutdown_debugging) {
			launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** Shutdown debugging is enabled. ***");
		}

		if (launchd_log_shutdown) {
			launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** Shutdown logging is enabled. ***");
		}

		if (launchd_log_perf) {
			launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** Performance logging is enabled. ***");
		}

		if (launchd_log_debug) {
			launchd_syslog(LOG_NOTICE | LOG_CONSOLE, "*** Debug logging is enabled. ***");
		}

		handle_pid1_crashes_separately();

		/* Start the update thread.
		 *
		 * <rdar://problem/5039559&6153301>
		 */
		pthread_t t = NULL;
		(void)os_assumes_zero(pthread_create(&t, NULL, update_thread, NULL));
		(void)os_assumes_zero(pthread_detach(t));

		/* PID 1 doesn't have a flat namespace. */
		launchd_flat_mach_namespace = false;
		fflush(launchd_console);
	} else {
		launchd_uid = getuid();
		launchd_var_available = true;
		if (asprintf(&launchd_label, "com.apple.launchd.peruser.%u", launchd_uid) == 0) {
			launchd_label = "com.apple.launchd.peruser.unknown";
		}

		struct passwd *pwent = getpwuid(launchd_uid);
		if (pwent) {
			launchd_username = strdup(pwent->pw_name);
		} else {
			launchd_username = "(unknown)";
		}

		if (asprintf(&_launchd_database_dir, LAUNCHD_DB_PREFIX "/com.apple.launchd.peruser.%u", launchd_uid) == 0) {
			_launchd_database_dir = "";
		}

		if (asprintf(&_launchd_log_dir, LAUNCHD_LOG_PREFIX "/com.apple.launchd.peruser.%u", launchd_uid) == 0) {
			_launchd_log_dir = "";
		}

		if (launchd_allow_global_dyld_envvars) {
			launchd_syslog(LOG_WARNING, "Per-user launchd will allow DYLD_* environment variables in the global environment.");
		}

		ipc_server_init();
		launchd_log_push();

		auditinfo_addr_t auinfo;
		if (posix_assumes_zero(getaudit_addr(&auinfo, sizeof(auinfo))) != -1) {
			launchd_audit_session = auinfo.ai_asid;
			launchd_syslog(LOG_DEBUG, "Our audit session ID is %i", launchd_audit_session);
		}

		launchd_audit_port = _audit_session_self();
		vproc_transaction_begin(NULL);
		vproc_transaction_end(NULL, NULL);
		launchd_syslog(LOG_DEBUG, "Per-user launchd started (UID/username): %u/%s.", launchd_uid, launchd_username);
	}

	monitor_networking_state();
	if (pid1_magic) {
		pd_pid1_prepare_legacy_ipc();
		ipc_server_init();
	}
	jobmgr_init(sflag);
	if (pid1_magic) {
		/* PureDarwin: mount /dev before launchctl bootstraps the system domain. */
		extern void pd_launchd_boot(void);
		pd_launchd_boot();

		/* iokit project: pd_launchd_load_daemons_dir() is real PureDarwin
		 * source (pd_launchd_plist.c) but was never actually called from
		 * anywhere upstream -- this project has no launchctl client to load
		 * jobs after the fact, so without this call launchd boots with an
		 * empty job table and nothing (console login, syslogd, ...) ever
		 * starts. Load system daemons directly at boot, matching real
		 * launchd's own pre-launchctl-load-message-era behavior. */
		extern void pd_launchd_load_daemons_dir(const char *dir);
		pd_launchd_load_daemons_dir("/Library/LaunchDaemons");
	}
	launchd_runtime_init2();
	launchd_runtime();
}

void
handle_pid1_crashes_separately(void)
{
	struct sigaction fsa;

	fsa.sa_sigaction = fatal_signal_handler;
	fsa.sa_flags = SA_SIGINFO;
	sigemptyset(&fsa.sa_mask);

	(void)posix_assumes_zero(sigaction(SIGILL, &fsa, NULL));
	(void)posix_assumes_zero(sigaction(SIGFPE, &fsa, NULL));
	(void)posix_assumes_zero(sigaction(SIGBUS, &fsa, NULL));
	(void)posix_assumes_zero(sigaction(SIGTRAP, &fsa, NULL));
	(void)posix_assumes_zero(sigaction(SIGABRT, &fsa, NULL));
	(void)posix_assumes_zero(sigaction(SIGSEGV, &fsa, NULL));
}

void
pd_pid1_prepare_legacy_ipc(void)
{
	if (mkdir("/var", 0755) < 0 && errno != EEXIST) {
		launchd_syslog(LOG_ERR | LOG_CONSOLE, "mkdir(\"/var\"): %s", strerror(errno));
	}
	if (mkdir(_PATH_VARTMP, 01777) < 0 && errno != EEXIST) {
		launchd_syslog(LOG_ERR | LOG_CONSOLE, "mkdir(\"%s\"): %s", _PATH_VARTMP, strerror(errno));
	}
}

void *
update_thread(void *nothing __attribute__((unused)))
{
	(void)posix_assumes_zero(setiopolicy_np(IOPOL_TYPE_DISK, IOPOL_SCOPE_THREAD, IOPOL_THROTTLE));

	while (launchd_sync_frequency) {
		sync();
		sleep(launchd_sync_frequency);
	}

	launchd_syslog(LOG_DEBUG, "Update thread exiting.");
	return NULL;
}

#define PID1_CRASH_LOGFILE "/var/log/launchd-pid1.crash"

/* This hack forces the dynamic linker to resolve these symbols ASAP */
static __attribute__((unused)) typeof(sync) *__junk_dyld_trick1 = sync;
static __attribute__((unused)) typeof(sleep) *__junk_dyld_trick2 = sleep;
static __attribute__((unused)) typeof(reboot) *__junk_dyld_trick3 = reboot;

void
do_pid1_crash_diagnosis_mode(const char *msg)
{
	if (launchd_wsp) {
		kill(launchd_wsp, SIGKILL);
		sleep(3);
		launchd_wsp = 0;
	}

	while (launchd_shutdown_debugging && !do_pid1_crash_diagnosis_mode2(msg)) {
		sleep(1);
	}
}

int
basic_fork(void)
{
	int wstatus = 0;
	pid_t p;

	switch ((p = fork())) {
	case -1:
		launchd_syslog(LOG_ERR | LOG_CONSOLE, "Can't fork PID 1 copy for crash debugging: %m");
		return p;
	case 0:
		return p;
	default:
		do {
			(void)waitpid(p, &wstatus, 0);
		} while(!WIFEXITED(wstatus));

		fprintf(stdout, "PID 1 copy: exit status: %d\n", WEXITSTATUS(wstatus));

		return 1;
	}

	return -1;
}

bool
do_pid1_crash_diagnosis_mode2(const char *msg)
{
	if (basic_fork() == 0) {
		/* Neuter our bootstrap port so that the shell doesn't try talking to us
		 * while we're blocked waiting on it.
		 */
		if (launchd_console) {
			fflush(launchd_console);
		}

		task_set_bootstrap_port(mach_task_self(), MACH_PORT_NULL);
		if (basic_fork() != 0) {
			if (launchd_console) {
				fflush(launchd_console);
			}

			return true;
		}
	} else {
		return true;
	}

	int fd;
	revoke(_PATH_CONSOLE);
	if ((fd = open(_PATH_CONSOLE, O_RDWR)) == -1) {
		_exit(2);
	}
	if (login_tty(fd) == -1) {
		_exit(3);
	}

	setenv("TERM", "vt100", 1);
	fprintf(stdout, "\n");
	fprintf(stdout, "Entering launchd PID 1 debugging mode...\n");
	fprintf(stdout, "The PID 1 launchd has crashed %s.\n", msg);
	fprintf(stdout, "It has fork(2)ed itself for debugging.\n");
	fprintf(stdout, "To debug the crashing thread of PID 1:\n");
	fprintf(stdout, "    gdb attach %d\n", getppid());
	fprintf(stdout, "To exit this shell and shut down:\n");
	fprintf(stdout, "    kill -9 1\n");
#if 0
	fprintf(stdout, "A sample of PID 1 has been written to %s\n", PID1_CRASH_LOGFILE);
#endif
	fprintf(stdout, "\n");
	fflush(stdout);

	execl(_PATH_BSHELL, "-sh", NULL);
	syslog(LOG_ERR, "can't exec %s for PID 1 crash debugging: %m", _PATH_BSHELL);
	_exit(EXIT_FAILURE);
}

void
fatal_signal_handler(int sig, siginfo_t *si, void *uap __attribute__((unused)))
{
	const char *doom_why = "at instruction";
	char msg[128];
#if 0
	char *sample_args[] = { "/usr/bin/sample", "1", "1", "-file", PID1_CRASH_LOGFILE, NULL };
	pid_t sample_p;
	int wstatus;
#endif

	crash_addr = si->si_addr;
	crash_pid = si->si_pid;
#if 0
	setenv("XPC_SERVICES_UNAVAILABLE", "1", 0);
	unlink(PID1_CRASH_LOGFILE);

	switch ((sample_p = vfork())) {
	case 0:
		execve(sample_args[0], sample_args, environ);
		_exit(EXIT_FAILURE);
		break;
	default:
		waitpid(sample_p, &wstatus, 0);
		break;
	case -1:
		break;
	}
#endif
	switch (sig) {
	default:
	case 0:
		break;
	case SIGBUS:
	case SIGSEGV:
		doom_why = "trying to read/write";
	case SIGILL:
	case SIGFPE:
	case SIGTRAP:
		snprintf(msg, sizeof(msg), "%s: %p (%s sent by PID %u)", doom_why, crash_addr, strsignal(sig), crash_pid);
		sync();
		do_pid1_crash_diagnosis_mode(msg);
		sleep(3);
		reboot(0);
		break;
	}
}

void
pid1_magic_init(void)
{
	launchd_label = "com.apple.launchd";
	launchd_username = "system";

	_launchd_database_dir = LAUNCHD_DB_PREFIX "/com.apple.launchd";
	_launchd_log_dir = LAUNCHD_LOG_PREFIX "/com.apple.launchd";

	(void)posix_assumes_zero(setsid());
	(void)posix_assumes_zero(chdir("/"));
	(void)posix_assumes_zero(setlogin("root"));

#if !TARGET_OS_EMBEDDED
	auditinfo_addr_t auinfo = {
		.ai_termid = {
			.at_type = AU_IPv4
		},
		.ai_asid = AU_ASSIGN_ASID,
		.ai_auid = AU_DEFAUDITID,
		.ai_flags = 0 /* because we don't have bsm sessions AU_SESSION_FLAG_IS_INITIAL*/,
	};

	if (setaudit_addr(&auinfo, sizeof(auinfo)) == -1) {
		launchd_syslog(LOG_WARNING | LOG_CONSOLE, "Could not set audit session: %d: %s.", errno, strerror(errno));
		_exit(EXIT_FAILURE);
	}

	launchd_audit_session = auinfo.ai_asid;
	launchd_syslog(LOG_DEBUG, "Audit Session ID: %i", launchd_audit_session);

	launchd_audit_port = _audit_session_self();
#endif // !TARGET_OS_EMBEDDED
}

char *
launchd_copy_persistent_store(int type, const char *file)
{
	char *result = NULL;
	if (!file) {
		file = "";
	}

	switch (type) {
	case LAUNCHD_PERSISTENT_STORE_DB:
		(void)asprintf(&result, "%s/%s", _launchd_database_dir, file);
		break;
	case LAUNCHD_PERSISTENT_STORE_LOGS:
		(void)asprintf(&result, "%s/%s", _launchd_log_dir, file);
		break;
	default:
		break;
	}

	return result;
}

int
_fd(int fd)
{
	if (fd >= 0) {
		(void)posix_assumes_zero(fcntl(fd, F_SETFD, 1));
	}
	return fd;
}

void
launchd_shutdown(void)
{
	int64_t now;

	if (launchd_shutting_down) {
		return;
	}

	runtime_ktrace0(RTKT_LAUNCHD_EXITING);

	launchd_shutting_down = true;
	launchd_log_push();

	now = runtime_get_wall_time();

	char *term_who = pid1_magic ? "System shutdown" : "Per-user launchd termination for ";
	launchd_syslog(LOG_INFO, "%s%s began", term_who, pid1_magic ? "" : launchd_username);

	os_assert(jobmgr_shutdown(root_jobmgr) != NULL);

#if HAVE_LIBAUDITD
	if (pid1_magic) {
//        (void)os_assumes_zero(audit_quick_stop()); _sjc_ linker can't find audit_quick_stop()
	}
#endif
}

void
launchd_SessionCreate(void)
{
#if !TARGET_OS_EMBEDDED
	auditinfo_addr_t auinfo = {
		.ai_termid = { .at_type = AU_IPv4 },
		.ai_asid = AU_ASSIGN_ASID,
		.ai_auid = getuid(),
		.ai_flags = 0,
	};
	if (setaudit_addr(&auinfo, sizeof(auinfo)) == 0) {
		char session[16];
		snprintf(session, sizeof(session), "%x", auinfo.ai_asid);
		setenv("SECURITYSESSIONID", session, 1);
	} else {
		launchd_syslog(LOG_WARNING, "Could not set audit session: %d: %s.", errno, strerror(errno));
	}
#endif // !TARGET_OS_EMBEDDED
}

void
testfd_or_openfd(int fd, const char *path, int flags)
{
	int tmpfd;

	if (-1 != (tmpfd = dup(fd))) {
		(void)posix_assumes_zero(runtime_close(tmpfd));
	} else {
		if (-1 == (tmpfd = open(path, flags | O_NOCTTY, DEFFILEMODE))) {
			launchd_syslog(LOG_ERR, "open(\"%s\", ...): %m", path);
		} else if (tmpfd != fd) {
			(void)posix_assumes_zero(dup2(tmpfd, fd));
			(void)posix_assumes_zero(runtime_close(tmpfd));
		}
	}
}

bool
get_network_state(void)
{
	struct ifaddrs *ifa, *ifai;
	bool up = false;
	int r;

	/* Workaround 4978696: getifaddrs() reports false ENOMEM */
	while ((r = getifaddrs(&ifa)) == -1 && errno == ENOMEM) {
		launchd_syslog(LOG_DEBUG, "Worked around bug: 4978696");
		(void)posix_assumes_zero(sched_yield());
	}

	if (posix_assumes_zero(r) == -1) {
		return network_up;
	}

	for (ifai = ifa; ifai; ifai = ifai->ifa_next) {
		if (!(ifai->ifa_flags & IFF_UP)) {
			continue;
		}
		if (ifai->ifa_flags & IFF_LOOPBACK) {
			continue;
		}
		if (ifai->ifa_addr->sa_family != AF_INET && ifai->ifa_addr->sa_family != AF_INET6) {
			continue;
		}
		up = true;
		break;
	}

	freeifaddrs(ifa);

	return up;
}

void
monitor_networking_state(void)
{
	int pfs = _fd(socket(PF_SYSTEM, SOCK_RAW, SYSPROTO_EVENT));
	struct kev_request kev_req;

	network_up = get_network_state();

	if (pfs == -1) {
		(void)os_assumes_zero(errno);
		return;
	}

	memset(&kev_req, 0, sizeof(kev_req));
	kev_req.vendor_code = KEV_VENDOR_APPLE;
	kev_req.kev_class = KEV_NETWORK_CLASS;

	if (posix_assumes_zero(ioctl(pfs, SIOCSKEVFILT, &kev_req)) == -1) {
		runtime_close(pfs);
		return;
	}

	(void)posix_assumes_zero(kevent_mod(pfs, EVFILT_READ, EV_ADD, 0, 0, &kqpfsystem_callback));
}

void
pfsystem_callback(void *obj __attribute__((unused)), struct kevent *kev)
{
	bool new_networking_state;
	char buf[1024];

	(void)posix_assumes_zero(read((int)kev->ident, &buf, sizeof(buf)));

	new_networking_state = get_network_state();

	if (new_networking_state != network_up) {
		network_up = new_networking_state;
		jobmgr_dispatch_all_semaphores(root_jobmgr);
	}
}
