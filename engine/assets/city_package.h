/**
 * @file city_package.h
 * @brief The cooked city package the runtime loads (ADR-005): a manifest
 * with provenance and a spawn, and chunk files holding parts, each a
 * mesh with a material name and an optional collider. Written by the
 * generator, read by the game and the validator.
 */
#ifndef ENGINE_ASSETS_CITY_PACKAGE_H_
#define ENGINE_ASSETS_CITY_PACKAGE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aabb.h"
#include "mesh_data.h"

#define CITY_PACKAGE_VERSION 1
#define CITY_PACKAGE_TEXT 128

typedef enum { PART_COLLIDER_NONE, PART_COLLIDER_MESH } part_collider_t;

typedef struct {
  char material[32];
  part_collider_t collider;
  int layer;      /* collision_layer_t value */
  float friction;
  uint64_t source_id;
  mesh_data_t mesh;
} city_part_t;

typedef struct {
  int column;
  int row;
  aabb_t bounds;
  city_part_t* parts;
  size_t part_count;
  size_t part_capacity;
  char file[CITY_PACKAGE_TEXT];
} city_chunk_t;

typedef struct {
  char name[CITY_PACKAGE_TEXT];
  char source[256];
  char source_date[32];
  char licence[CITY_PACKAGE_TEXT];
  char generated[32];
  double bounds_lat_lon[4];
  double origin_lat, origin_lon;
  float chunk_size;
  vec3_t spawn_position;
  float spawn_yaw;
  uint64_t config_hash;
  int generator_version;
  city_chunk_t* chunks;
  size_t chunk_count;
  size_t chunk_capacity;
} city_package_t;

city_package_t city_package_create(void);
void city_package_free(city_package_t* package);
city_chunk_t* city_package_chunk(city_package_t* package, int column, int row);
city_part_t* city_chunk_add_part(city_chunk_t* chunk, const char* material, part_collider_t collider,
                                 int layer, float friction, uint64_t source_id);
bool city_package_write(const city_package_t* package, const char* directory);
bool city_package_read(city_package_t* package, const char* directory);
size_t city_package_triangle_count(const city_package_t* package);

#endif  // ENGINE_ASSETS_CITY_PACKAGE_H_
