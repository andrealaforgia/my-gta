/**
 * @file city_generator.h
 * @brief Intermediate city model to cooked package: road ribbons,
 * junction patches, pavements with kerbs, extruded buildings, ground
 * tiles, chunking, a spawn on a road, provenance. Deterministic for the
 * same model and settings.
 */
#ifndef TOOLS_COMMON_CITY_GENERATOR_H_
#define TOOLS_COMMON_CITY_GENERATOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "city_model.h"
#include "city_package.h"

typedef struct {
  float chunk_size;
  float road_height;
  bool pavements;
  float pavement_width;
  float kerb_height;
  float road_texture_length;
  float window_width;
  float storey_height;
  int facade_variants;
  float ground_uv_scale;
  float road_friction;
  float pavement_friction;
  float ground_friction;
  float building_friction;
  float lamp_spacing; /* metres between lamp posts, 0 for none */
  float tree_spacing; /* metres between trees in parks, 0 for none */
} city_generator_options_t;

typedef struct {
  size_t roads;
  size_t junction_patches;
  size_t buildings;
  size_t buildings_unclean; /* roofs that needed the fan fallback */
  size_t pavements;
  size_t ground_tiles;
  size_t lamps;
  size_t trees;
  size_t areas;
} city_generator_stats_t;

city_generator_options_t city_generator_default_options(void);
bool city_generator_options_load(city_generator_options_t* options,
                                 const char* path);
uint64_t city_generator_options_hash(const city_generator_options_t* options);
bool city_generate(city_package_t* package, const city_model_t* model,
                   const city_generator_options_t* options, const char* name,
                   city_generator_stats_t* stats);

#endif  // TOOLS_COMMON_CITY_GENERATOR_H_
