#if defined(STM32G4)
#include "stm32g4xx_hal.h"
#else
#error "No known STM32 variant"
#endif
#include "log.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

//#define LOG_USE_UNICODE
static struct {
  void *udata;
  log_LockFn lock;
  int level;
  bool quiet;
} L;


#if defined(LOG_USE_UNICODE)
static const char *level_strings[] = {
  u8"· T", u8"◇ D", u8"ℹ I ", u8"⚠ W ", u8"● E", u8"✗ F"
};
#else
static const char *level_strings[] = {
  "TRC", "DBG", "INF", "WRN", "ERR", "FTL"
};
#endif

static const char *level_colors[] = {
  "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m"
};


void log_set_lock(log_LockFn fn) {
  L.lock = fn;
}


void log_set_level(int level) {
  L.level = level;
}


void log_set_quiet(bool enable) {
  L.quiet = enable;
}


void log_log(int level, char const* file, int line, char const* fmt, ...) {

  if (L.quiet || level < L.level) return;
  va_list ap;
  va_start(ap, fmt);
  if (L.lock) { L.lock(true); }
  fprintf(stdout, "%lu %s%s\x1b[0m \x1b[90m%-8.8s:%03d:\x1b[0m ", (unsigned long)HAL_GetTick(), level_colors[level], level_strings[level], file, line);
  vfprintf(stdout, fmt, ap);
  fprintf(stdout, "\r\n");
  fflush(stdout);
  if (L.lock) { L.lock(false); }
  va_end(ap);
}
