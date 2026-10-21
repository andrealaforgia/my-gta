/**
 * @file osm_reader.h
 * @brief A limited OpenStreetMap XML reader (ADR-004): nodes, ways with
 * node references and tags, relations counted and kept as tags only.
 * Source ids are preserved. Nothing here is used at runtime.
 */
#ifndef TOOLS_COMMON_OSM_READER_H_
#define TOOLS_COMMON_OSM_READER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define OSM_KEY_LENGTH 64
#define OSM_VALUE_LENGTH 128

typedef struct {
  char key[OSM_KEY_LENGTH];
  char value[OSM_VALUE_LENGTH];
} osm_tag_t;

typedef enum {
  OSM_FURNITURE_NONE,
  OSM_FURNITURE_BENCH,
  OSM_FURNITURE_BUS_STOP,
  OSM_FURNITURE_BOLLARD,
  OSM_FURNITURE_SIGNALS,
  OSM_FURNITURE_CROSSING /* a pedestrian crossing on a road */
} osm_furniture_t;

typedef struct {
  int64_t id;
  double lat;
  double lon;
  bool tree; /* tagged natural=tree */
  /* Bench, bus stop, bollard or signals: the only other node tags kept. */
  osm_furniture_t furniture;
} osm_node_t;

typedef struct {
  int64_t id;
  int64_t* node_refs;
  size_t node_count;
  osm_tag_t* tags;
  size_t tag_count;
} osm_way_t;

typedef enum {
  OSM_MEMBER_NODE,
  OSM_MEMBER_WAY,
  OSM_MEMBER_RELATION
} osm_member_type_t;

typedef struct {
  osm_member_type_t type;
  int64_t ref;
  char role[32];
} osm_member_t;

typedef struct {
  int64_t id;
  osm_tag_t* tags;
  size_t tag_count;
  osm_member_t* members;
  size_t member_count;
} osm_relation_t;

typedef struct {
  double min_lat, min_lon, max_lat, max_lon;
} osm_bounds_t;

typedef struct {
  osm_node_t* nodes;
  size_t node_count;
  osm_way_t* ways;
  size_t way_count;
  osm_relation_t* relations;
  size_t relation_count;
  osm_bounds_t bounds;
  bool has_bounds;
} osm_data_t;

bool osm_load(osm_data_t* data, const char* path);
bool osm_parse(osm_data_t* data, const char* xml);
/* Drops the nodes lying more than `margin_metres` outside the extract's
 * bounds, so a river or a motorway that runs on for kilometres does not
 * stretch the city; ways simply lose those points. Does nothing without
 * bounds. */
void osm_clip_to_bounds(osm_data_t* data, double margin_metres);
void osm_free(osm_data_t* data);
const osm_node_t* osm_find_node(const osm_data_t* data, int64_t id);
const osm_way_t* osm_find_way(const osm_data_t* data, int64_t id);
const osm_relation_t* osm_find_relation(const osm_data_t* data, int64_t id);
const char* osm_tag(const osm_tag_t* tags, size_t count, const char* key);

#endif  // TOOLS_COMMON_OSM_READER_H_
