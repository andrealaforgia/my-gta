/**
 * @file city_menu.h
 * @brief The start-up screen where the player picks the test track or one
 * of the cooked cities.
 */
#ifndef GAME_SRC_MAIN_CITY_MENU_H_
#define GAME_SRC_MAIN_CITY_MENU_H_

#include "city_list.h"
#include "renderer.h"
#include "window.h"

/* Shows "Test track" and the cities and waits for a choice. Returns -1 for
 * the test track, the index into `cities` for a city, or -2 if the player
 * quit. `selected` is the entry the cursor starts on (0 is the track). */
int city_menu_run(window_t* window, renderer_t* renderer,
                  const city_entry_t* cities, size_t count, int selected);

/* How the entries are laid out, for tests: the title of row `row`, where row
 * 0 is the test track. */
const char* city_menu_row_title(const city_entry_t* cities, size_t count,
                                size_t row);

/* The row the cursor lands on after a key, wrapping round the ends. */
size_t city_menu_move(size_t row, size_t rows, menu_key_t key);

#endif  // GAME_SRC_MAIN_CITY_MENU_H_
