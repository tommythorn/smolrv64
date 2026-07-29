// Fork-based checkpoint server for tb_virtio (DPI).
//
// The ubuntu wedge repro is deterministic but costs ~5.5h of sim to reach; this makes
// each experiment cost only the post-checkpoint cycles. At a chosen cycle the testbench
// calls ckpt_wait_cmd(): the process becomes a SERVER that blocks polling for a command
// file. For each command it fork()s; the CHILD redirects stdout to the command's log
// file and returns the command words to Verilog (which enables the requested trace and
// keeps simulating -- the full simulator state came along with the fork). The PARENT
// waits for the child, then polls for the next command. Verilator here is single-
// threaded (sim report: "1 threads"), so fork is safe.
//
// Command file (default /tmp/probe-ckpt-cmd, override +ckpt_cmd=<path>): one line
//   <log-path> <watch_lo-hex> <watch_hi-hex> <extra-cycles-dec> [<scan_val-hex>]
// e.g.  /tmp/exp1.log 80345000 80346000 300000000
// The file is unlinked once read. watch_lo=watch_hi=0 disables the write-watch.
// A nonzero scan_val makes the child scan all of DDR for that 64-bit value first
// (prints the PAs holding it at the checkpoint). "quit" exits the server.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/wait.h>

extern "C" int ckpt_wait_cmd(const char* cmd_path,
                             long long* watch_lo, long long* watch_hi,
                             long long* extra_cycles, long long* scan_val) {
    for (;;) {
        FILE* f = fopen(cmd_path, "r");
        if (!f) { sleep(2); continue; }
        char log_path[512] = {0};
        unsigned long long lo = 0, hi = 0, cyc = 0, sval = 0;
        char first[512] = {0};
        int n = fscanf(f, "%511s", first);
        if (n == 1 && strcmp(first, "quit") == 0) { fclose(f); unlink(cmd_path); return 0; }
        strncpy(log_path, first, sizeof(log_path) - 1);
        n = fscanf(f, "%llx %llx %llu %llx", &lo, &hi, &cyc, &sval);
        fclose(f);
        unlink(cmd_path);
        if (n < 3 || !log_path[0]) {
            fprintf(stderr, "[ckpt] malformed command, ignored\n");
            continue;
        }
        fflush(stdout); fflush(stderr);
        pid_t pid = fork();
        if (pid < 0) { perror("[ckpt] fork"); continue; }
        if (pid == 0) {
            // child: own log, run the experiment
            if (!freopen(log_path, "w", stdout)) _exit(2);
            fprintf(stderr, "[ckpt] child %d -> %s watch=[%llx,%llx) cycles=%llu\n",
                    getpid(), log_path, lo, hi, cyc);
            *watch_lo = (long long)lo; *watch_hi = (long long)hi;
            *extra_cycles = (long long)cyc; *scan_val = (long long)sval;
            return 1;
        }
        int st = 0;
        waitpid(pid, &st, 0);
        fprintf(stderr, "[ckpt] child %d done (status %d); waiting for next command\n", pid, st);
    }
}
