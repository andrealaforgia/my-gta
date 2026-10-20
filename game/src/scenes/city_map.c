#include "city_map.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  const char* material;
  uint8_t r, g, b;
} layer_t;

/* Painted in this order, later over earlier. */
static const layer_t layers[] = {
    {"park", 96, 140, 84},         {"water", 84, 128, 170},
    {"parking", 120, 120, 124},    {"paving", 196, 184, 160},
    {"pavement", 170, 166, 158},   {"road", 250, 250, 248},
    {"junction", 250, 250, 248},   {"roof", 150, 92, 76},
    {"roof_slate", 110, 112, 122}, {"roof_flat", 130, 130, 132},
};

static void fill_triangle(city_map_t* map, float ax, float az, float bx,
                          float bz, float cx, float cz, const layer_t* layer) {
  float scale = (float)map->size / map->extent;
  float x[3] = {(ax - map->min_x) * scale, (bx - map->min_x) * scale,
                (cx - map->min_x) * scale};
  float y[3] = {(az - map->min_z) * scale, (bz - map->min_z) * scale,
                (cz - map->min_z) * scale};
  float min_x = fminf(x[0], fminf(x[1], x[2])),
        max_x = fmaxf(x[0], fmaxf(x[1], x[2]));
  float min_y = fminf(y[0], fminf(y[1], y[2])),
        max_y = fmaxf(y[0], fmaxf(y[1], y[2]));
  int x0 = (int)fmaxf(floorf(min_x), 0),
      x1 = (int)fminf(ceilf(max_x), (float)map->size - 1);
  int y0 = (int)fmaxf(floorf(min_y), 0),
      y1 = (int)fminf(ceilf(max_y), (float)map->size - 1);
  float denominator =
      (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2]);
  if (fabsf(denominator) < 1e-9f) return;
  for (int py = y0; py <= y1; py++)
    for (int px = x0; px <= x1; px++) {
      float qx = (float)px + 0.5f, qy = (float)py + 0.5f;
      float l0 = ((y[1] - y[2]) * (qx - x[2]) + (x[2] - x[1]) * (qy - y[2])) /
                 denominator;
      float l1 = ((y[2] - y[0]) * (qx - x[2]) + (x[0] - x[2]) * (qy - y[2])) /
                 denominator;
      float l2 = 1.0f - l0 - l1;
      if (l0 < -0.001f || l1 < -0.001f || l2 < -0.001f) continue;
      uint8_t* p = map->pixels + ((size_t)py * map->size + px) * 4;
      p[0] = layer->r, p[1] = layer->g, p[2] = layer->b, p[3] = 255;
    }
}

bool city_map_build(city_map_t* map, const city_package_t* package, int size) {
  memset(map, 0, sizeof(*map));
  if (size < 16 || package->chunk_count == 0) return false;
  /* The picture frames what is built, not the ground that runs out past it. */
  aabb_t bounds = aabb_empty();
  for (size_t c = 0; c < package->chunk_count; c++)
    for (size_t p = 0; p < package->chunks[c].part_count; p++) {
      const city_part_t* part = &package->chunks[c].parts[p];
      if (strcmp(part->material, "ground") != 0 && part->mesh.vertex_count > 0)
        bounds = aabb_merge(bounds, mesh_data_bounds(&part->mesh));
    }
  float width = bounds.max.x - bounds.min.x,
        depth = bounds.max.z - bounds.min.z;
  if (!(width > 0) || !(depth > 0)) return false;
  map->extent = fmaxf(width, depth) * 1.02f;
  map->min_x = bounds.min.x - (map->extent - width) * 0.5f;
  map->min_z = bounds.min.z - (map->extent - depth) * 0.5f;
  map->size = size;
  map->pixels = malloc((size_t)size * size * 4);
  if (!map->pixels) {
    map->size = 0;
    return false;
  }
  for (size_t i = 0; i < (size_t)size * size; i++) {
    uint8_t* p = map->pixels + i * 4;
    p[0] = 58, p[1] = 58, p[2] = 62, p[3] = 255;
  }
  for (size_t l = 0; l < sizeof(layers) / sizeof(layers[0]); l++)
    for (size_t c = 0; c < package->chunk_count; c++) {
      const city_chunk_t* chunk = &package->chunks[c];
      for (size_t p = 0; p < chunk->part_count; p++) {
        const city_part_t* part = &chunk->parts[p];
        if (strcmp(part->material, layers[l].material) != 0) continue;
        const mesh_data_t* mesh = &part->mesh;
        for (size_t i = 0; i + 2 < mesh->index_count; i += 3) {
          vec3_t a = mesh->vertices[mesh->indices[i]].position;
          vec3_t b = mesh->vertices[mesh->indices[i + 1]].position;
          vec3_t d = mesh->vertices[mesh->indices[i + 2]].position;
          fill_triangle(map, a.x, a.z, b.x, b.z, d.x, d.z, &layers[l]);
        }
      }
    }
  return true;
}

void city_map_free(city_map_t* map) {
  free(map->pixels);
  memset(map, 0, sizeof(*map));
}

bool city_map_to_pixel(const city_map_t* map, float x, float z, int* px,
                       int* py) {
  if (map->size == 0) return false;
  float scale = (float)map->size / map->extent;
  float fx = (x - map->min_x) * scale, fy = (z - map->min_z) * scale;
  if (fx < 0 || fy < 0 || fx >= (float)map->size || fy >= (float)map->size)
    return false;
  *px = (int)fx;
  *py = (int)fy;
  return true;
}

void city_map_window(const city_map_t* map, float x, float z, float radius,
                     float* u0, float* v0, float* u1, float* v1) {
  float span = 2.0f * radius / map->extent;
  if (span >= 1.0f) {
    *u0 = 0, *v0 = 0, *u1 = 1, *v1 = 1;
    return;
  }
  float u = (x - map->min_x) / map->extent, v = (z - map->min_z) / map->extent;
  float lo_u = u - span * 0.5f, lo_v = v - span * 0.5f;
  if (lo_u < 0) lo_u = 0;
  if (lo_v < 0) lo_v = 0;
  if (lo_u > 1.0f - span) lo_u = 1.0f - span;
  if (lo_v > 1.0f - span) lo_v = 1.0f - span;
  *u0 = lo_u, *v0 = lo_v, *u1 = lo_u + span, *v1 = lo_v + span;
}
