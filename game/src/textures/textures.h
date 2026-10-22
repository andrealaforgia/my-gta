/**
 * @file textures.h
 * @brief Procedural RGBA textures so the prototype needs no image assets.
 * Deterministic from their seed.
 */
#ifndef GAME_SRC_TEXTURES_TEXTURES_H_
#define GAME_SRC_TEXTURES_TEXTURES_H_

#include <stdint.h>

#include "renderer.h"

#define TEXTURE_SIZE 512
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
  texture_handle_t facade_fresco; /* painted fronts along the Corso */
  texture_handle_t facade_brick;  /* the Municipio */
  texture_handle_t paving; /* stone flags for squares and pedestrian streets */
  texture_handle_t door;   /* a panelled street door in its frame */
  texture_handle_t shopfront; /* ground-floor shops on main streets */
  texture_handle_t roof_slate;
  texture_handle_t roof_flat;
  texture_handle_t bark;    /* tree trunks */
  texture_handle_t foliage; /* tree crowns and hedges */
  texture_handle_t stone;   /* rubble walls and cornices */
  texture_handle_t planks;  /* fences */
  texture_handle_t metal;   /* lamp posts, poles, rails, furniture */
  texture_handle_t signs;   /* shop signs: SIGN_WORDS words on boards */
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
uint8_t* texture_facade_fresco(unsigned seed);
uint8_t* texture_facade_brick(unsigned seed);
uint8_t* texture_shopfront(unsigned seed);
uint8_t* texture_paving(unsigned seed);
uint8_t* texture_door(unsigned seed);
uint8_t* texture_roof_slate(unsigned seed);
uint8_t* texture_roof_flat(unsigned seed);
uint8_t* texture_bark(unsigned seed);
uint8_t* texture_foliage(unsigned seed);
uint8_t* texture_stone(unsigned seed);
uint8_t* texture_planks(unsigned seed);
uint8_t* texture_metal(unsigned seed);
/* An atlas of shop signs, two boards across and eight down, each a word in
 * capital letters on a painted board: SIGN_WORDS of them, row by row. */
#define SIGN_WORDS 16
uint8_t* texture_signs(unsigned seed);
const char* sign_word(int index);
uint8_t* texture_checker(void);

texture_set_t texture_set_upload(renderer_t* renderer);

#endif  // GAME_SRC_TEXTURES_TEXTURES_H_
