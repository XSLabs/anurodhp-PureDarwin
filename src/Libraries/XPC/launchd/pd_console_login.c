/*
 * PureDarwin console login helper.
 *
 * launchd opens StandardInPath with O_NOCTTY, so a direct /bin/zsh
 * LaunchDaemon has file descriptors for /dev/console but no controlling tty.
 * Keep the tty setup out of PID 1: launchd supervises this helper, and this
 * helper creates the console session then execs real login(1)
 * (/usr/bin/login, third_party/system_cmds/login.tproj -- DAR-164), which
 * does the actual username+password prompt/account lookup/credential drop
 * and only then execs the authenticated account's real shell. Before
 * DAR-164 this execed /bin/bash directly as root with zero prompt at all.
 *
 * The tty setup follows the classic getty dance so it is robust on a serial
 * console (serial=3): open NON-BLOCKING so we never wedge in the tty open
 * waiting for carrier (DCD) - that carrier-wait is exactly the intermittent
 * "sometimes I get a shell, sometimes I don't" boot hang - then force CLOCAL
 * (ignore carrier), clear O_NONBLOCK, and acquire the controlling terminal.
 *
 * DAR-165 (real getty -> login -> shell chain): the default exec target is
 * now real /bin/getty (third_party/system_cmds/getty.tproj, built by
 * tools/userland_staging/build_getty.sh), not login directly. getty owns
 * the interactive part of real Apple's architecture -- printing the
 * hostname/"login:" banner and reading a username -- then does its own
 * real, unmodified execle(LO, "login", "-p1"/"-fp1", name, ..., env)
 * handoff (third_party/system_cmds/getty.tproj/main.c) to the exact same
 * real /usr/bin/login DAR-164 already proved.
 *
 * Real DEVIATION from genuine Apple boot architecture, and why: real Apple
 * launchd (com.apple.getty.plist, third_party/system_cmds/getty.tproj/
 * com.apple.getty.plist) invokes getty directly on /dev/console with
 * ProgramArguments ["/usr/libexec/getty", "std.9600", "console"] -- getty
 * itself does chown/chmod/revoke + a *blocking* open() of the tty device
 * (opentty(), getty.tproj/main.c) with no non-blocking/CLOCAL handling
 * unless a gettytab "nc" flag is set. That blocking open on a carrier-
 * sensitive device is EXACTLY the mechanism this file's own header (above)
 * documents fixing as a real, previously-observed intermittent boot hang.
 * Handing /dev/console to launchd->getty directly would very likely
 * reintroduce that regression. So this helper keeps doing its own
 * already-fixed, already-QEMU-serial-tuned tty acquisition (non-blocking
 * open, force CLOCAL, drop O_NONBLOCK, TIOCSCTTY) and then execs getty in
 * real getty's own documented "old style" mode (argv[2] == "-": "the file
 * descriptors are already set up for us", see main.c's own comment on that
 * exact branch) so getty's real opentty()/chown/revoke/blocking-open path
 * is skipped entirely -- getty reuses the fd 0/1/2 already wired up here.
 * This is a real, load-bearing engineering trade-off (documented here, not
 * guessed): genuine getty->login exec semantics for the part that matters
 * (the interactive login: prompt + real login(1) handoff), without
 * regressing a real, already-fixed hang in the part that doesn't need to
 * move (tty acquisition on this port's specific serial console).
 *
 * Fallback during the transition (per DAR-165's own explicit instruction:
 * keep the ability to skip the new getty layer until it's QEMU-verified
 * end to end): pass argv[1] == "-direct-login" to skip getty and exec
 * /usr/bin/login directly instead, exactly DAR-164's already-committed
 * behavior. org.puredarwin.console-login-direct.plist (Disabled, not
 * loaded by default -- see inject_into_sd_image.sh) selects this mode;
 * flip it to the active LaunchDaemon (and disable the getty one) to fall
 * back without rebuilding anything. Once the real getty chain is
 * QEMU-verified (bad password rejected, good password reaches an
 * authenticated shell), this fallback plist/flag can be retired.
 */
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

static int g_trace_fd = -1;

static void
ctrace(const char *msg)
{
	if (g_trace_fd >= 0) {
		(void)write(g_trace_fd, msg, strlen(msg));
	}
}

int
main(int argc, char *argv[])
{
	const char *tty = "/dev/console";
	int direct_login = (argc > 1 && strcmp(argv[1], "-direct-login") == 0);

	g_trace_fd = open(tty, O_WRONLY | O_NOCTTY | O_NONBLOCK);
	ctrace("pd-console-login: start\n");

	if (setsid() < 0 && errno != EPERM) {
		ctrace("pd-console-login: setsid failed\n");
		_exit(126);
	}
	ctrace("pd-console-login: setsid ok\n");

	/* Non-blocking open: never wait on carrier. */
	int fd = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		ctrace("pd-console-login: open failed\n");
		_exit(126);
	}
	ctrace("pd-console-login: open ok\n");

	/* Ignore modem carrier so reads/writes and the controlling-tty grab don't
	 * stall when DCD is deasserted on the serial line. */
	struct termios t;
	if (tcgetattr(fd, &t) == 0) {
		t.c_cflag |= CLOCAL;
		(void)tcsetattr(fd, TCSANOW, &t);
		ctrace("pd-console-login: CLOCAL set\n");
	} else {
		ctrace("pd-console-login: tcgetattr failed (continuing)\n");
	}

	/* Back to blocking for normal shell I/O. */
	int fl = fcntl(fd, F_GETFL, 0);
	if (fl >= 0) {
		(void)fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
	}

	if (ioctl(fd, TIOCSCTTY, 0) < 0) {
		ctrace("pd-console-login: TIOCSCTTY failed (continuing)\n");
	} else {
		ctrace("pd-console-login: TIOCSCTTY ok\n");
	}

	(void)dup2(fd, STDIN_FILENO);
	(void)dup2(fd, STDOUT_FILENO);
	(void)dup2(fd, STDERR_FILENO);
	if (fd > STDERR_FILENO) {
		(void)close(fd);
	}

	/* iokit project / DAR-164: this used to setenv() a hardcoded root
	 * identity and exec /bin/bash directly -- zero username/password
	 * prompt at all. Real login(1) (third_party/system_cmds/login.tproj,
	 * see tools/userland_staging/build_login.sh) now does that real
	 * prompt+auth (crypt(3) against real /etc/master.passwd, same
	 * account DB pwtool provisions/edits) and, on success, sets
	 * LOGNAME/USER/HOME/SHELL/PATH itself for the REAL authenticated
	 * account -- not always root -- and execs that account's real
	 * shell. PATH/TERM are still set here as login(1)'s own real
	 * pre-auth environment (real login.c preserves TERM across its
	 * `environ = envinit` reset when not passed -p; PATH gets
	 * overwritten unconditionally by login.c itself once an account is
	 * known, real _PATH_STDPATH/_PATH_DEFPATH by uid, so this PATH
	 * value is only ever visible to login.c's own pre-auth code path,
	 * never to the eventual shell). XDG_RUNTIME_DIR is deliberately no
	 * longer set here -- real login.c's own `environ = envinit` wipes it
	 * before the shell exec regardless (matches real login(1) semantics,
	 * not a regression); a per-account XDG_RUNTIME_DIR is a real,
	 * separate follow-up if ever needed.
	 */
	setenv("PATH", "/bin:/sbin:/usr/bin:/usr/sbin", 1);
	setenv("TERM", "vt220", 0);

	if (direct_login) {
		/* DAR-165 fallback path: skip getty, exec login directly.
		 * Exactly DAR-164's original behavior, kept available via
		 * org.puredarwin.console-login-direct.plist until the real
		 * getty chain below is QEMU-verified end to end. */
		char *login_argv[] = { "/usr/bin/login", NULL };
		ctrace("pd-console-login: exec /usr/bin/login (direct, fallback)\n");
		execv(login_argv[0], login_argv);
		ctrace("pd-console-login: execv login failed\n");
		_exit(127);
	}

	/* DAR-165 primary path: hand off to real getty in its own "old
	 * style" mode (argv[2] == "-") so it reuses the controlling tty
	 * already set up above instead of re-opening/revoking /dev/console
	 * itself (see this file's header comment for why). getty prints
	 * the login: prompt, reads a username, and does its own real
	 * execle(LO, "login", ...) handoff to the same /usr/bin/login. */
	char *getty_argv[] = { "getty", "default", "-", NULL };
	ctrace("pd-console-login: exec /bin/getty\n");
	execv("/bin/getty", getty_argv);
	ctrace("pd-console-login: execv getty failed\n");
	_exit(127);
}
