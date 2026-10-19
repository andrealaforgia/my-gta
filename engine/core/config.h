/**
 * @file config.h
 * @brief Key = value text configuration. Lines starting with # and blank
 * lines are ignored; a # after the value starts a comment. Lookups with a
 * default count how often the default was used so tuning files can be
 * checked for missing keys.
 */
#ifndef ENGINE_CORE_CONFIG_H_
#define ENGINE_CORE_CONFIG_H_

#include <stdbool.h>
#include <stddef.h>

#define CONFIG_KEY_LENGTH 64
#define CONFIG_VALUE_LENGTH 256

typedef struct {
  char key[CONFIG_KEY_LENGTH];
  char value[CONFIG_VALUE_LENGTH];
} config_entry_t;

typedef struct {
  config_entry_t* entries;
  size_t count;
  size_t capacity;
  size_t malformed;
  size_t defaulted;
} config_t;

config_t config_parse(const char* text);
/* Returns false and leaves an empty config if the file cannot be read. */
bool config_load(config_t* config, const char* path);
void config_free(config_t* config);

size_t config_count(const config_t* config);
size_t config_malformed_count(const config_t* config);
size_t config_defaulted_count(const config_t* config);
bool config_has(const config_t* config, const char* key);
float config_get_float(config_t* config, const char* key, float fallback);
int config_get_int(config_t* config, const char* key, int fallback);
bool config_get_bool(config_t* config, const char* key, bool fallback);
const char* config_get_string(config_t* config, const char* key, const char* fallback);

#endif  // ENGINE_CORE_CONFIG_H_
