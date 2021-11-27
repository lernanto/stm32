/**
 *
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>


__attribute__((weak)) size_t _log_write(const void *buf, size_t len)
{
    return (size_t)fwrite(buf, 1, len, stderr);
}

__attribute__((weak, noreturn)) void _log_abort(void)
{
    abort();
}
