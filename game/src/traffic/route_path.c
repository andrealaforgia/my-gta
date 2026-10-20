#include "route_path.h"

#include <math.h>
#include <stdlib.h>

bool route_path_init(route_path_t* path, const city_route_t* route,
                     physics_world_t* physics) {
  path->route = route;
  path->cumulative = calloc(route->point_count, sizeof(float));
  path->ground = calloc(route->point_count, sizeof(float));
  if (!path->cumulative || !path->ground) {
    route_path_free(path);
    return false;
  }
  for (size_t k = 1; k < route->point_count; k++)
    path->cumulative[k] =
        path->cumulative[k - 1] +
        hypotf(route->points[2 * k] - route->points[2 * (k - 1)],
               route->points[2 * k + 1] - route->points[2 * (k - 1) + 1]);
  path->length = path->cumulative[route->point_count - 1];
  for (size_t k = 0; k < route->point_count; k++) {
    ray_t down = {vec3(route->points[2 * k], 300.0f, route->points[2 * k + 1]),
                  vec3(0, -1, 0)};
    ray_cast_result_t hit;
    path->ground[k] =
        physics_ray_cast(physics, down, 600.0f, COLLISION_LAYER_GROUND, &hit)
            ? hit.hit.point.y
            : 0.0f;
  }
  return true;
}

void route_path_free(route_path_t* path) {
  free(path->cumulative);
  free(path->ground);
  path->cumulative = NULL;
  path->ground = NULL;
}

void route_path_sample(const route_path_t* path, float distance, int direction,
                       float* x, float* z, float* dx, float* dz,
                       float* ground) {
  const city_route_t* route = path->route;
  float along = direction > 0 ? distance : path->length - distance;
  if (along < 0) along = 0;
  if (along > path->length) along = path->length;
  size_t k = 1;
  while (k + 1 < route->point_count && path->cumulative[k] < along) k++;
  float a = path->cumulative[k - 1], b = path->cumulative[k];
  float t = b > a ? (along - a) / (b - a) : 0;
  float x0 = route->points[2 * (k - 1)], z0 = route->points[2 * (k - 1) + 1];
  float x1 = route->points[2 * k], z1 = route->points[2 * k + 1];
  *x = x0 + (x1 - x0) * t;
  *z = z0 + (z1 - z0) * t;
  float length = hypotf(x1 - x0, z1 - z0);
  float ux = length > 1e-4f ? (x1 - x0) / length : 1;
  float uz = length > 1e-4f ? (z1 - z0) / length : 0;
  *dx = direction > 0 ? ux : -ux;
  *dz = direction > 0 ? uz : -uz;
  *ground = path->ground[k - 1] + (path->ground[k] - path->ground[k - 1]) * t;
}
