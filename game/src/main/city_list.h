/**
 * @file city_list.h
 * @brief The cooked cities on disk, for the start-up menu: every directory
 * under the cities folder that holds a package manifest.
 */
#ifndef GAME_SRC_MAIN_CITY_LIST_H_
#define GAME_SRC_MAIN_CITY_LIST_H_

#include <stddef.h>

#define CITY_LIST_MAX 32

typedef struct {
  char name[48];  /* the directory name, "barletta" */
  char title[48]; /* for the screen, "Barletta" */
  char path[200]; /* the package directory */
} city_entry_t;

/* Fills `out` with up to `max` cities found in `directory`, sorted by name,
 * and returns how many. A missing directory has none. */
size_t city_list(const char* directory, city_entry_t* out, size_t max);

#endif  // GAME_SRC_MAIN_CITY_LIST_H_
