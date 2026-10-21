#include "city_lidar.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

static void local_to_utm(const city_model_t* model, double x, double z,
                         double* e, double* n) {
  double lat, lon;
  local_point_t p = {x, z};
  projection_unproject(&model->projection, p, &lat, &lon);
  utm33_forward(lat, lon, e, n);
}

lidar_window_t city_model_utm_window(const city_model_t* model, double margin) {
  /* The model's own extent: its geographic bounds, corner by corner. */
  const osm_bounds_t* b = &model->bounds;
  double lats[2] = {b->min_lat, b->max_lat}, lons[2] = {b->min_lon, b->max_lon};
  lidar_window_t w = {1e30, 1e30, -1e30, -1e30};
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) {
      double e, n;
      utm33_forward(lats[i], lons[j], &e, &n);
      w.west = fmin(w.west, e), w.east = fmax(w.east, e);
      w.south = fmin(w.south, n), w.north = fmax(w.north, n);
    }
  w.west -= margin, w.south -= margin, w.east += margin, w.north += margin;
  return w;
}

bool city_model_apply_dtm(city_model_t* model, const lidar_set_t* dtm,
                          double spacing) {
  const osm_bounds_t* b = &model->bounds;
  local_point_t corner_a =
      projection_project(&model->projection, b->max_lat, b->min_lon);
  local_point_t corner_b =
      projection_project(&model->projection, b->min_lat, b->max_lon);
  const double margin = 40.0;
  double min_x = fmin(corner_a.x, corner_b.x) - margin;
  double max_x = fmax(corner_a.x, corner_b.x) + margin;
  double min_z = fmin(corner_a.z, corner_b.z) - margin;
  double max_z = fmax(corner_a.z, corner_b.z) + margin;
  int cols = (int)ceil((max_x - min_x) / spacing) + 1;
  int rows = (int)ceil((max_z - min_z) / spacing) + 1;
  if (cols < 2 || rows < 2 || cols > 4096 || rows > 4096) return false;
  float* heights = malloc((size_t)cols * (size_t)rows * sizeof(float));
  if (!heights) return false;
  size_t found = 0;
  for (int j = 0; j < rows; j++)
    for (int i = 0; i < cols; i++) {
      double e, n;
      local_to_utm(model, min_x + i * spacing, min_z + j * spacing, &e, &n);
      float v;
      if (lidar_set_sample(dtm, e, n, &v)) {
        heights[j * cols + i] = v, found++;
      } else {
        heights[j * cols + i] = NAN;
      }
    }
  if (found < (size_t)cols * (size_t)rows / 2) {
    LOG_ERROR("lidar", "the terrain model covers only %zu of %d samples", found,
              cols * rows);
    free(heights);
    return false;
  }
  /* Holes take the mean of what is around them, spreading inwards. */
  for (int pass = 0; pass < 64; pass++) {
    bool any = false;
    for (int j = 0; j < rows; j++)
      for (int i = 0; i < cols; i++) {
        if (!isnan(heights[j * cols + i])) continue;
        double sum = 0;
        int n = 0;
        for (int dj = -1; dj <= 1; dj++)
          for (int di = -1; di <= 1; di++) {
            int jj = j + dj, ii = i + di;
            if (jj < 0 || jj >= rows || ii < 0 || ii >= cols) continue;
            float v = heights[jj * cols + ii];
            if (!isnan(v)) sum += v, n++;
          }
        if (n > 0)
          heights[j * cols + i] = (float)(sum / n);
        else
          any = true;
      }
    if (!any) break;
  }
  float lowest = heights[0];
  for (int i = 1; i < cols * rows; i++) lowest = fminf(lowest, heights[i]);
  for (int i = 0; i < cols * rows; i++) heights[i] -= lowest;
  free(model->terrain.heights);
  model->terrain.min_x = min_x;
  model->terrain.min_z = min_z;
  model->terrain.dx = spacing;
  model->terrain.dz = spacing;
  model->terrain.cols = cols;
  model->terrain.rows = rows;
  model->terrain.heights = heights;
  float highest = 0;
  for (int i = 0; i < cols * rows; i++) highest = fmaxf(highest, heights[i]);
  LOG_INFO("lidar", "terrain: %d x %d samples every %.1f m, %.1f m of relief",
           cols, rows, spacing, (double)highest);
  return true;
}

static bool inside_ring(const local_point_t* ring, size_t count, double x,
                        double z) {
  bool inside = false;
  for (size_t i = 0, j = count - 1; i < count; j = i++)
    if (((ring[i].z > z) != (ring[j].z > z)) &&
        x < (ring[j].x - ring[i].x) * (z - ring[i].z) /
                    (ring[j].z - ring[i].z) +
                ring[i].x)
      inside = !inside;
  return inside;
}

static double edge_distance(const local_point_t* ring, size_t count, double x,
                            double z) {
  double best = 1e30;
  for (size_t i = 0; i < count; i++) {
    local_point_t a = ring[i], b = ring[(i + 1) % count];
    double dx = b.x - a.x, dz = b.z - a.z, l2 = dx * dx + dz * dz;
    double t = l2 > 1e-12 ? ((x - a.x) * dx + (z - a.z) * dz) / l2 : 0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    best = fmin(best, hypot(x - (a.x + t * dx), z - (a.z + t * dz)));
  }
  return best;
}

static int compare_floats(const void* a, const void* b) {
  float x = *(const float*)a, y = *(const float*)b;
  return x < y ? -1 : (x > y ? 1 : 0);
}

size_t city_model_apply_dsm(city_model_t* model, const lidar_set_t* dsm,
                            const lidar_set_t* dtm) {
  size_t changed = 0;
  for (size_t b = 0; b < model->building_count; b++) {
    city_building_t* building = &model->buildings[b];
    if (building->height_source == HEIGHT_TAGGED ||
        building->height_source == HEIGHT_FROM_LEVELS ||
        building->point_count < 3)
      continue;
    double min_x = 1e30, max_x = -1e30, min_z = 1e30, max_z = -1e30;
    for (size_t i = 0; i < building->point_count; i++) {
      min_x = fmin(min_x, building->points[i].x);
      max_x = fmax(max_x, building->points[i].x);
      min_z = fmin(min_z, building->points[i].z);
      max_z = fmax(max_z, building->points[i].z);
    }
    /* Samples a metre apart, at least a metre and a half inside the walls so
     * a footprint a little out of register does not read the street. */
    double step = fmax(1.0, sqrt((max_x - min_x) * (max_z - min_z) / 400.0));
    size_t capacity = 1024, n = 0;
    float* samples = malloc(capacity * sizeof(float));
    if (!samples) return changed;
    for (double z = min_z; z <= max_z; z += step)
      for (double x = min_x; x <= max_x; x += step) {
        if (!inside_ring(building->points, building->point_count, x, z) ||
            edge_distance(building->points, building->point_count, x, z) < 1.5)
          continue;
        bool in_hole = false;
        for (size_t h = 0; h < building->hole_count && !in_hole; h++)
          in_hole = inside_ring(building->holes[h].points,
                                building->holes[h].count, x, z);
        if (in_hole) continue;
        double e, nn;
        local_to_utm(model, x, z, &e, &nn);
        float top, ground;
        if (!lidar_set_sample(dsm, e, nn, &top) ||
            !lidar_set_sample(dtm, e, nn, &ground))
          continue;
        if (n == capacity) {
          float* grown = realloc(samples, capacity * 2 * sizeof(float));
          if (!grown) break;
          samples = grown;
          capacity *= 2;
        }
        samples[n++] = top - ground;
      }
    if (n >= 6) {
      qsort(samples, n, sizeof(float), compare_floats);
      /* The eaves: low in the footprint, where a pitched roof comes down. */
      float eaves = samples[n / 4];
      if (eaves < 3.0f) eaves = 3.0f;
      if (eaves > 90.0f) eaves = 90.0f;
      if (building->height_source == HEIGHT_DEFAULT &&
          model->stats.heights_defaulted > 0)
        model->stats.heights_defaulted--;
      building->height = eaves;
      building->height_source = HEIGHT_FROM_LIDAR;
      changed++;
    }
    free(samples);
  }
  LOG_INFO("lidar", "%zu building heights read from the surface model",
           changed);
  return changed;
}
