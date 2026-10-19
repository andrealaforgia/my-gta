/**
 * @file log.h
 * @brief Logging with a severity and a subsystem label, to stderr. Kept
 * free of SDL so the offline tools share it.
 */
#ifndef ENGINE_CORE_LOG_H_
#define ENGINE_CORE_LOG_H_

typedef enum { LOG_LEVEL_DEBUG, LOG_LEVEL_INFO, LOG_LEVEL_WARN, LOG_LEVEL_ERROR } log_level_t;

void log_set_level(log_level_t minimum);
void log_message(log_level_t level, const char* subsystem, const char* format, ...);

#define LOG_DEBUG(subsystem, ...) log_message(LOG_LEVEL_DEBUG, subsystem, __VA_ARGS__)
#define LOG_INFO(subsystem, ...) log_message(LOG_LEVEL_INFO, subsystem, __VA_ARGS__)
#define LOG_WARN(subsystem, ...) log_message(LOG_LEVEL_WARN, subsystem, __VA_ARGS__)
#define LOG_ERROR(subsystem, ...) log_message(LOG_LEVEL_ERROR, subsystem, __VA_ARGS__)

#endif  // ENGINE_CORE_LOG_H_
