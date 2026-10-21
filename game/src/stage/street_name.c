#include "street_name.h"

#include <ctype.h>
#include <math.h>
#include <string.h>

bool street_name_at(const city_route_t* routes, size_t count, float x, float z,
                    char* out, size_t size) {
  float best = 1e30f;
  const city_route_t* nearest = NULL;
  for (size_t r = 0; r < count; r++) {
    const city_route_t* route = &routes[r];
    if (route->name[0] == '\0') continue;
    float reach = route->width * 0.5f + route->pavement + 2.0f;
    for (size_t k = 1; k < route->point_count; k++) {
      float ax = route->points[2 * (k - 1)],
            az = route->points[2 * (k - 1) + 1];
      float bx = route->points[2 * k], bz = route->points[2 * k + 1];
      float dx = bx - ax, dz = bz - az, length2 = dx * dx + dz * dz;
      float t = length2 > 1e-6f ? ((x - ax) * dx + (z - az) * dz) / length2 : 0;
      t = t < 0 ? 0 : (t > 1 ? 1 : t);
      float d = hypotf(x - (ax + t * dx), z - (az + t * dz));
      /* Within reach, and closer than what is nearest so far. */
      if (d < reach && d < best) best = d, nearest = route;
    }
  }
  if (!nearest || size == 0) {
    if (size > 0) out[0] = '\0';
    return false;
  }
  size_t i = 0;
  for (; nearest->name[i] && i + 1 < size; i++) {
    char c = nearest->name[i];
    out[i] = c == '_' ? ' ' : (char)toupper((unsigned char)c);
  }
  out[i] = '\0';
  return true;
}
