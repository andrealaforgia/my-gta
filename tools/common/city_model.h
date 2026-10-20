/**
 * @file city_model.h
 * @brief The intermediate city representation (ADR-005): road segments
 * split at junctions, junctions with their degree, building footprints,
 * provenance and import statistics. Written and read as line-oriented
 * text in a directory: roads.txt, buildings.txt, manifest.txt.
 */
#ifndef TOOLS_COMMON_CITY_MODEL_H_
#define TOOLS_COMMON_CITY_MODEL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "osm_reader.h"
#include "projection.h"

#define CITY_MODEL_VERSION 3
#define CITY_TEXT_LENGTH 64

typedef enum {
  ROAD_CLASS_MOTORWAY,
  ROAD_CLASS_PRIMARY,
  ROAD_CLASS_SECONDARY,
  ROAD_CLASS_TERTIARY,
  ROAD_CLASS_RESIDENTIAL,
  ROAD_CLASS_SERVICE,
  ROAD_CLASS_UNCLASSIFIED,
  ROAD_CLASS_PEDESTRIAN, /* a paved street for people; the car may use it */
  ROAD_CLASS_PATH,       /* footway, cycleway, path, steps: a narrow strip */
  ROAD_CLASS_WATERWAY,   /* a stream or canal drawn as a water ribbon */
  ROAD_CLASS_COUNT
} road_class_t;

typedef enum {
  AREA_WATER,
  AREA_PARK,    /* parks, gardens, grass, woods */
  AREA_PARKING, /* car parks */
  AREA_SQUARE,  /* pedestrian areas and squares */
  AREA_KIND_COUNT
} area_kind_t;

typedef enum {
  HEIGHT_TAGGED,
  HEIGHT_FROM_LEVELS,
  HEIGHT_DEFAULT,         /* a deterministic guess, nothing better known */
  HEIGHT_FROM_CATEGORY,   /* houses and huts have typical heights */
  HEIGHT_FROM_NEIGHBOURS, /* the median of tagged buildings nearby */
} height_source_t;

typedef struct {
  int64_t source_id;
  road_class_t class;
  float width;
  bool width_inferred;
  bool oneway;
  int layer;
  bool bridge;
  bool tunnel;
  char surface[CITY_TEXT_LENGTH];
  char name[CITY_TEXT_LENGTH];
  local_point_t* points;
  int64_t* node_ids;
  size_t point_count;
} city_road_t;

typedef struct {
  int64_t node_id;
  local_point_t position;
  int degree;
} city_junction_t;

typedef struct {
  local_point_t* points;
  size_t count;
} city_ring_t;

typedef struct {
  int64_t source_id;
  float height;
  height_source_t height_source;
  int levels;
  char category[CITY_TEXT_LENGTH];
  /* Closed ring, last point not repeated, anticlockwise from above. */
  local_point_t* points;
  size_t point_count;
  city_ring_t* holes; /* courtyards, clockwise from above */
  size_t hole_count;
} city_building_t;

typedef struct {
  int64_t source_id;
  area_kind_t kind;
  local_point_t* points; /* anticlockwise from above, not repeated */
  size_t point_count;
} city_area_t;

typedef struct {
  size_t ways_total;
  size_t roads_accepted;
  size_t ways_ignored;
  size_t roads_degenerate;
  size_t buildings_accepted;
  size_t buildings_malformed;
  size_t relations_skipped;
  size_t areas_accepted;
  size_t rings_unclosed;
  size_t ends_snapped;
  size_t widths_inferred;
  size_t heights_defaulted;
  size_t tunnels_skipped; /* underpasses and building passages */
  size_t roads_merged;    /* segments joined end to end through a bend */
  size_t bends_rounded;   /* interior corners replaced by arcs */
} city_stats_t;

/* Ground height above the lowest sampled point on a regular grid of local
 * metres: column i is at x = min_x + i * dx, row j at z = min_z + j * dz
 * (row 0 is the north edge). cols == 0 means flat ground at 0. */
typedef struct {
  double min_x, min_z, dx, dz;
  int cols, rows;
  float* heights; /* rows * cols, row-major */
} city_terrain_t;

typedef struct {
  city_road_t* roads;
  size_t road_count;
  city_junction_t* junctions;
  size_t junction_count;
  city_building_t* buildings;
  size_t building_count;
  city_area_t* areas;
  size_t area_count;
  city_terrain_t terrain;
  projection_t projection;
  osm_bounds_t bounds;
  char source[256];
  char import_date[32];
  city_stats_t stats;
} city_model_t;

/* Bilinear height at a local point; clamped to the edge outside the grid,
 * 0 when there is no terrain. */
double city_terrain_height(const city_terrain_t* terrain, double x, double z);
/* Reads a lat/lon lattice of ground elevations (tools/fetch_elevation.py),
 * projects it onto the model's local grid, smooths it a little and makes
 * it relative to its lowest point. */
bool city_model_load_elevation(city_model_t* model, const char* path);

const char* road_class_name(road_class_t class);
const char* area_kind_name(area_kind_t kind);
bool area_kind_from_name(const char* name, area_kind_t* kind);
bool road_class_from_name(const char* name, road_class_t* class);

void city_model_free(city_model_t* model);
bool city_model_write(const city_model_t* model, const char* directory);
bool city_model_read(city_model_t* model, const char* directory);

/* Growth helpers used by the importer. */
city_road_t* city_model_add_road(city_model_t* model);
city_building_t* city_model_add_building(city_model_t* model);
city_junction_t* city_model_add_junction(city_model_t* model);
city_area_t* city_model_add_area(city_model_t* model);
/* Takes ownership of the points. */
bool city_building_add_hole(city_building_t* building, local_point_t* points,
                            size_t count);

#endif  // TOOLS_COMMON_CITY_MODEL_H_
