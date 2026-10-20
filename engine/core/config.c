#include "config.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void trim(char* text) {
  char* start = text;
  while (*start && isspace((unsigned char)*start)) start++;
  size_t length = strlen(start);
  while (length > 0 && isspace((unsigned char)start[length - 1])) length--;
  memmove(text, start, length);
  text[length] = '\0';
}

static void strip_comment(char* text) {
  char* hash = strchr(text, '#');
  if (hash) *hash = '\0';
}

static bool append(config_t* config, const char* key, const char* value) {
  if (config->count == config->capacity) {
    size_t capacity = config->capacity ? config->capacity * 2 : 16;
    config_entry_t* grown =
        realloc(config->entries, capacity * sizeof(config_entry_t));
    if (!grown) return false;
    config->entries = grown;
    config->capacity = capacity;
  }
  config_entry_t* entry = &config->entries[config->count++];
  snprintf(entry->key, sizeof(entry->key), "%s", key);
  snprintf(entry->value, sizeof(entry->value), "%s", value);
  return true;
}

static void parse_line(config_t* config, char* line) {
  strip_comment(line);
  trim(line);
  if (line[0] == '\0') return;
  char* equals = strchr(line, '=');
  if (!equals) {
    config->malformed++;
    return;
  }
  *equals = '\0';
  char* key = line;
  char* value = equals + 1;
  trim(key);
  trim(value);
  if (key[0] == '\0') {
    config->malformed++;
    return;
  }
  append(config, key, value);
}

config_t config_parse(const char* text) {
  config_t config = {NULL, 0, 0, 0, 0};
  char line[CONFIG_KEY_LENGTH + CONFIG_VALUE_LENGTH];
  size_t length = 0;
  for (const char* cursor = text;; cursor++) {
    if (*cursor == '\n' || *cursor == '\0') {
      line[length] = '\0';
      parse_line(&config, line);
      length = 0;
      if (*cursor == '\0') break;
    } else if (length < sizeof(line) - 1) {
      line[length++] = *cursor;
    }
  }
  return config;
}

bool config_load(config_t* config, const char* path) {
  config_t empty = {NULL, 0, 0, 0, 0};
  *config = empty;
  FILE* file = fopen(path, "rb");
  if (!file) return false;
  fseek(file, 0, SEEK_END);
  long size = ftell(file);
  fseek(file, 0, SEEK_SET);
  if (size < 0) {
    fclose(file);
    return false;
  }
  char* text = malloc((size_t)size + 1);
  if (!text) {
    fclose(file);
    return false;
  }
  size_t read = fread(text, 1, (size_t)size, file);
  fclose(file);
  text[read] = '\0';
  *config = config_parse(text);
  free(text);
  return true;
}

void config_free(config_t* config) {
  free(config->entries);
  config->entries = NULL;
  config->count = config->capacity = 0;
}

size_t config_count(const config_t* config) { return config->count; }
size_t config_malformed_count(const config_t* config) {
  return config->malformed;
}
size_t config_defaulted_count(const config_t* config) {
  return config->defaulted;
}

static const char* find(const config_t* config, const char* key) {
  for (size_t i = 0; i < config->count; i++)
    if (strcmp(config->entries[i].key, key) == 0)
      return config->entries[i].value;
  return NULL;
}

bool config_has(const config_t* config, const char* key) {
  return find(config, key) != NULL;
}

const char* config_get_string(config_t* config, const char* key,
                              const char* fallback) {
  const char* value = find(config, key);
  if (!value) {
    config->defaulted++;
    return fallback;
  }
  return value;
}

float config_get_float(config_t* config, const char* key, float fallback) {
  const char* value = find(config, key);
  if (!value) {
    config->defaulted++;
    return fallback;
  }
  char* end;
  float parsed = strtof(value, &end);
  return end == value ? fallback : parsed;
}

int config_get_int(config_t* config, const char* key, int fallback) {
  const char* value = find(config, key);
  if (!value) {
    config->defaulted++;
    return fallback;
  }
  char* end;
  long parsed = strtol(value, &end, 10);
  return end == value ? fallback : (int)parsed;
}

bool config_get_bool(config_t* config, const char* key, bool fallback) {
  const char* value = find(config, key);
  if (!value) {
    config->defaulted++;
    return fallback;
  }
  return strcmp(value, "true") == 0 || strcmp(value, "1") == 0 ||
         strcmp(value, "yes") == 0;
}
