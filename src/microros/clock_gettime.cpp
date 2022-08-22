#include <mbed.h>
#include <sys/time.h>

extern "C" int clock_gettime(clockid_t unused, struct timespec *tp) {
    (void)unused;

    uint64_t ms = Kernel::get_ms_count();
    tp->tv_sec = ms / 1000;
    tp->tv_nsec = (ms % 1000) * 1000000;

    return 0;
}