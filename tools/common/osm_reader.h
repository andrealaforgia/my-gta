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

typedef struct {
  int64_t id;
  double lat;
  double lon;
} osm_node_t;

typedef struct {
  int64_t id;
  int64_t* node_refs;
  size_t node_count;
  osm_tag_t* tags;
  size_t tag_count;
} osm_way_t;

typedef struct {
  int64_t id;
  osm_tag_t* tags;
  size_t tag_count;
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
void osm_free(osm_data_t* data);
const osm_node_t* osm_find_node(const osm_data_t* data, int64_t id);
const osm_way_t* osm_find_way(const osm_data_t* data, int64_t id);
const char* osm_tag(const osm_tag_t* tags, size_t count, const char* key);

#endif  // TOOLS_COMMON_OSM_READER_H_
