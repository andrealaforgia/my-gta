#include "city_list.h"

#include <ctype.h>
#include <dirent.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static void make_title(const char* name, char* title, size_t size) {
  size_t n = 0;
  for (; name[n] && n + 1 < size; n++) {
    char c = name[n] == '_' || name[n] == '-' ? ' ' : name[n];
    bool word_start = n == 0 || title[n - 1] == ' ';
    title[n] = word_start ? (char)toupper((unsigned char)c) : c;
  }
  title[n] = '\0';
}

static int by_name(const void* a, const void* b) {
  return strcmp(((const city_entry_t*)a)->name, ((const city_entry_t*)b)->name);
}

size_t city_list(const char* directory, city_entry_t* out, size_t max) {
  DIR* dir = opendir(directory);
  if (!dir) return 0;
  size_t count = 0;
  struct dirent* item;
  while (count < max && (item = readdir(dir)) != NULL) {
    if (item->d_name[0] == '.' || strlen(item->d_name) >= sizeof(out->name))
      continue;
    char manifest[320];
    snprintf(manifest, sizeof(manifest), "%s/%s/manifest.txt", directory,
             item->d_name);
    struct stat info;
    if (stat(manifest, &info) != 0) continue;
    city_entry_t* entry = &out[count++];
    snprintf(entry->name, sizeof(entry->name), "%s", item->d_name);
    snprintf(entry->path, sizeof(entry->path), "%s/%s", directory,
             item->d_name);
    make_title(entry->name, entry->title, sizeof(entry->title));
  }
  closedir(dir);
  qsort(out, count, sizeof(*out), by_name);
  return count;
}
