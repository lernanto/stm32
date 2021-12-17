/**
 *
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>


#ifndef __GNUC__
#define __attribute__(attr)
#endif  /* __GNUC__ */


__attribute__((weak)) size_t _log_write(const void *buf, size_t len)
{
    return (size_t)fwrite(buf, 1, len, stderr);
}

__attribute__((weak, noreturn)) void _log_abort(void)
{
    abort();
}
