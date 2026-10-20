#include "log.h"

#include <stdarg.h>
#include <stdio.h>

static log_level_t minimum_level = LOG_LEVEL_INFO;

static const char* level_names[] = {"debug", "info", "warn", "error"};

void log_set_level(log_level_t minimum) { minimum_level = minimum; }

void log_message(log_level_t level, const char* subsystem, const char* format,
                 ...) {
  if (level < minimum_level) return;
  fprintf(stderr, "[%s] %s: ", level_names[level], subsystem);
  va_list arguments;
  va_start(arguments, format);
  vfprintf(stderr, format, arguments);
  va_end(arguments);
  fputc('\n', stderr);
}
