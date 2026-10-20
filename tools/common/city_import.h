/**
 * @file city_import.h
 * @brief OSM data to the intermediate city model: classify and project
 * roads, clean polylines, split ways at junctions, validate building
 * footprints and assign heights. Defaults are configurable and every
 * inferred value is marked as such.
 */
#ifndef TOOLS_COMMON_CITY_IMPORT_H_
#define TOOLS_COMMON_CITY_IMPORT_H_

#include "city_model.h"
#include "osm_reader.h"

typedef struct {
  float default_width[ROAD_CLASS_COUNT]; /* metres, used when no width tag */
  float storey_height;                   /* metres per building:levels */
  float default_height_min; /* untagged buildings vary between these */
  float default_height_max;
  float min_road_length; /* shorter segments are degenerate */
  float snap_distance;   /* join road ends this close to another road */
  float bend_radius; /* round interior corners with arcs up to this, 0 off */
  float neighbour_reach; /* untagged buildings take the median height nearby */
  bool include_service_roads;
} city_import_options_t;

city_import_options_t city_import_default_options(void);
bool city_import(city_model_t* model, const osm_data_t* osm,
                 city_import_options_t options, const char* source_path);

#endif  // TOOLS_COMMON_CITY_IMPORT_H_
