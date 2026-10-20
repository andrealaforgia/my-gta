/**
 * @file textures.h
 * @brief Procedural RGBA textures so the prototype needs no image assets.
 * Deterministic from their seed.
 */
#ifndef GAME_SRC_TEXTURES_TEXTURES_H_
#define GAME_SRC_TEXTURES_TEXTURES_H_

#include <stdint.h>

#include "renderer.h"

#define TEXTURE_SIZE 256
#define CITY_FACADE_STYLES 12

typedef struct {
  texture_handle_t asphalt;
  texture_handle_t road; /* asphalt with edge and dashed centre lines */
  texture_handle_t concrete;
  texture_handle_t grass;
  texture_handle_t earth; /* bare yards and courtyards between buildings */
  texture_handle_t facade;
  texture_handle_t facades[CITY_FACADE_STYLES];
  texture_handle_t roof;
  texture_handle_t roof_slate;
  texture_handle_t roof_flat;
  texture_handle_t checker;
} texture_set_t;

/* Callers free the returned buffer. */
uint8_t* texture_asphalt(unsigned seed);
uint8_t* texture_road(unsigned seed);
uint8_t* texture_concrete(unsigned seed);
uint8_t* texture_grass(unsigned seed);
uint8_t* texture_earth(unsigned seed);
uint8_t* texture_facade(unsigned seed, int window_rows, int window_columns);
uint8_t* texture_roof(unsigned seed);
uint8_t* texture_roof_slate(unsigned seed);
uint8_t* texture_roof_flat(unsigned seed);
uint8_t* texture_checker(void);

texture_set_t texture_set_upload(renderer_t* renderer);

#endif  // GAME_SRC_TEXTURES_TEXTURES_H_
