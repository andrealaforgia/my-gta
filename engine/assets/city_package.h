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
  int layer; /* collision_layer_t value */
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

#define CITY_PACKAGE_MAX_LABELS 48

/* A street name for the map, in capitals, anchored at its middle. */
typedef struct {
  float x, z;
  char text[32];
} city_label_t;

/* A road as people and cars use it: its centre line in local metres, its
 * carriageway width, whether cars may use it and the pavement beside it. */
typedef struct {
  int drivable;   /* 1 for roads cars use, 0 for streets and paths for people */
  int oneway;     /* 1 if cars may only go from the first point to the last */
  float width;    /* carriageway, metres */
  float pavement; /* each side, metres; 0 for none */
  int64_t start_node, end_node; /* the junctions it joins, OpenStreetMap ids */
  float* points;                /* x, z pairs */
  size_t point_count;
  char name[48]; /* the street's name, spaces as underscores; empty if none */
} city_route_t;

/* A place at the kerb where a car stands, and which way it faces. */
typedef struct {
  float x, z, yaw;
  int variant; /* picks a colour and a body */
} city_parking_t;

/* A traffic signal head: where it stands, the way the traffic it controls
 * is heading (a unit vector; 0, 0 for any), and which of the two phases it
 * keeps: group 0 is green while group 1 is red, and the other way round. */
/* Parts that make up a pole the car can knock down (a lamp post, a signal,
 * a road sign) carry a source id with the pole flag set, the pole's index
 * in bits 40 to 61 and the feature they belong to in the low 40 bits. */
#define CITY_POLE_FLAG (1ull << 62)
#define CITY_POLE_ORIGINAL_MASK ((1ull << 40) - 1)
static inline uint64_t city_pole_source(uint32_t index, uint64_t original) {
  return CITY_POLE_FLAG | ((uint64_t)index << 40) |
         (original & CITY_POLE_ORIGINAL_MASK);
}
static inline bool city_source_is_pole(uint64_t source) {
  return (source & CITY_POLE_FLAG) != 0;
}
static inline uint32_t city_source_pole_index(uint64_t source) {
  return (uint32_t)((source >> 40) & 0x3fffff);
}
static inline uint64_t city_source_original(uint64_t source) {
  return source & CITY_POLE_ORIGINAL_MASK;
}

typedef struct {
  float x, z;
  float heading_x, heading_z;
  int group;
} city_signal_t;

#define CITY_PACKAGE_MAX_AIRCRAFT 12

typedef enum { AIRCRAFT_PLANE, AIRCRAFT_HELICOPTER } aircraft_kind_t;

/* An aeroplane or helicopter standing at the airport, and which way it faces
 * (yaw as for the spawn). */
typedef struct {
  int kind; /* aircraft_kind_t */
  float x, y, z, yaw;
} city_aircraft_t;

/* The airport beside the city: the middle of its runway, the way the runway
 * lies (yaw of its first end, as for the spawn), its size in metres and the
 * gate, where the road from the city ends, and the height of the road there. */
typedef struct {
  bool present;
  float x, z, yaw, length, width;
  float gate_x, gate_y, gate_z;
} city_airport_t;

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
  city_label_t labels[CITY_PACKAGE_MAX_LABELS];
  size_t label_count;
  city_route_t* routes;
  size_t route_count;
  city_parking_t* parked;
  size_t parked_count;
  city_signal_t* signals;
  size_t signal_count;
  float* crossings; /* x, z pairs: zebra crossings on streets cars use */
  size_t crossing_count;
  float* bus_stops; /* x, z pairs: where buses stop */
  size_t bus_stop_count;
  city_airport_t airport;
  city_aircraft_t aircraft[CITY_PACKAGE_MAX_AIRCRAFT];
  size_t aircraft_count;
} city_package_t;

city_package_t city_package_create(void);
void city_package_free(city_package_t* package);
/* Takes ownership of the route's points. */
bool city_package_add_route(city_package_t* package, const city_route_t* route);
bool city_package_add_signal(city_package_t* package, city_signal_t signal);
bool city_package_add_crossing(city_package_t* package, float x, float z);
bool city_package_add_bus_stop(city_package_t* package, float x, float z);
bool city_package_add_parking(city_package_t* package, city_parking_t spot);
city_chunk_t* city_package_chunk(city_package_t* package, int column, int row);
city_part_t* city_chunk_add_part(city_chunk_t* chunk, const char* material,
                                 part_collider_t collider, int layer,
                                 float friction, uint64_t source_id);
bool city_package_write(const city_package_t* package, const char* directory);
bool city_package_read(city_package_t* package, const char* directory);
size_t city_package_triangle_count(const city_package_t* package);

#endif  // ENGINE_ASSETS_CITY_PACKAGE_H_
