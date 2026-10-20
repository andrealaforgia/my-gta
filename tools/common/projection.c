#include "projection.h"

#include <math.h>

#define EARTH_RADIUS_METRES 6371008.8

projection_t projection_create(double origin_lat, double origin_lon) {
  projection_t p;
  p.origin_lat = origin_lat;
  p.origin_lon = origin_lon;
  p.metres_per_degree_lat = EARTH_RADIUS_METRES * M_PI / 180.0;
  p.metres_per_degree_lon =
      p.metres_per_degree_lat * cos(origin_lat * M_PI / 180.0);
  return p;
}

projection_t projection_for_bounds(osm_bounds_t bounds) {
  return projection_create((bounds.min_lat + bounds.max_lat) * 0.5,
                           (bounds.min_lon + bounds.max_lon) * 0.5);
}

local_point_t projection_project(const projection_t* p, double lat,
                                 double lon) {
  local_point_t point;
  point.x = (lon - p->origin_lon) * p->metres_per_degree_lon;
  point.z = -(lat - p->origin_lat) * p->metres_per_degree_lat;
  return point;
}

void projection_unproject(const projection_t* p, local_point_t point,
                          double* lat, double* lon) {
  *lon = p->origin_lon + point.x / p->metres_per_degree_lon;
  *lat = p->origin_lat - point.z / p->metres_per_degree_lat;
}
