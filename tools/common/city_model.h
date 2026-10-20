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

#define CITY_MODEL_VERSION 1
#define CITY_TEXT_LENGTH 64

typedef enum {
  ROAD_CLASS_MOTORWAY,
  ROAD_CLASS_PRIMARY,
  ROAD_CLASS_SECONDARY,
  ROAD_CLASS_TERTIARY,
  ROAD_CLASS_RESIDENTIAL,
  ROAD_CLASS_SERVICE,
  ROAD_CLASS_UNCLASSIFIED,
  ROAD_CLASS_COUNT
} road_class_t;

typedef enum { HEIGHT_TAGGED, HEIGHT_FROM_LEVELS, HEIGHT_DEFAULT } height_source_t;

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
  int64_t source_id;
  float height;
  height_source_t height_source;
  int levels;
  char category[CITY_TEXT_LENGTH];
  local_point_t* points; /* closed ring, last point not repeated, anticlockwise from above */
  size_t point_count;
} city_building_t;

typedef struct {
  size_t ways_total;
  size_t roads_accepted;
  size_t ways_ignored;
  size_t roads_degenerate;
  size_t buildings_accepted;
  size_t buildings_malformed;
  size_t relations_skipped;
  size_t widths_inferred;
  size_t heights_defaulted;
} city_stats_t;

typedef struct {
  city_road_t* roads;
  size_t road_count;
  city_junction_t* junctions;
  size_t junction_count;
  city_building_t* buildings;
  size_t building_count;
  projection_t projection;
  osm_bounds_t bounds;
  char source[256];
  char import_date[32];
  city_stats_t stats;
} city_model_t;

const char* road_class_name(road_class_t class);
bool road_class_from_name(const char* name, road_class_t* class);

void city_model_free(city_model_t* model);
bool city_model_write(const city_model_t* model, const char* directory);
bool city_model_read(city_model_t* model, const char* directory);

/* Growth helpers used by the importer. */
city_road_t* city_model_add_road(city_model_t* model);
city_building_t* city_model_add_building(city_model_t* model);
city_junction_t* city_model_add_junction(city_model_t* model);

#endif  // TOOLS_COMMON_CITY_MODEL_H_
