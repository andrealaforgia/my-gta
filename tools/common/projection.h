/**
 * @file projection.h
 * @brief Geographic degrees to local metres: an equirectangular
 * projection about the region centre, accurate to well under a metre
 * across a few kilometres. x is east, z is south (ADR-007); north is -z.
 */
#ifndef TOOLS_COMMON_PROJECTION_H_
#define TOOLS_COMMON_PROJECTION_H_

#include "osm_reader.h"

typedef struct {
  double origin_lat;
  double origin_lon;
  double metres_per_degree_lat;
  double metres_per_degree_lon;
} projection_t;

typedef struct {
  double x;
  double z;
} local_point_t;

projection_t projection_create(double origin_lat, double origin_lon);
projection_t projection_for_bounds(osm_bounds_t bounds);
local_point_t projection_project(const projection_t* projection, double lat,
                                 double lon);
void projection_unproject(const projection_t* projection, local_point_t point,
                          double* lat, double* lon);

#endif  // TOOLS_COMMON_PROJECTION_H_
