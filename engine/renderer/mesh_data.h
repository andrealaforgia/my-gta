/**
 * @file mesh_data.h
 * @brief CPU-side triangle meshes: a growable vertex and index list with
 * builders for the primitives the prototype and the city generator use.
 * No GPU dependency, so the offline tools share it.
 */
#ifndef ENGINE_RENDERER_MESH_DATA_H_
#define ENGINE_RENDERER_MESH_DATA_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aabb.h"
#include "vec3.h"

typedef struct {
  vec3_t position;
  vec3_t normal;
  float u;
  float v;
} vertex_t;

typedef struct {
  vertex_t* vertices;
  size_t vertex_count;
  size_t vertex_capacity;
  uint32_t* indices;
  size_t index_count;
  size_t index_capacity;
} mesh_data_t;

mesh_data_t mesh_data_create(void);
void mesh_data_free(mesh_data_t* mesh);
bool mesh_data_add_vertex(mesh_data_t* mesh, vertex_t vertex);
bool mesh_data_add_triangle(mesh_data_t* mesh, uint32_t a, uint32_t b,
                            uint32_t c);
/* Anticlockwise a, b, c, d seen from the front; the normal is computed. */
bool mesh_data_add_quad(mesh_data_t* mesh, vec3_t a, vec3_t b, vec3_t c,
                        vec3_t d, float uv_scale);
bool mesh_data_add_box(mesh_data_t* mesh, vec3_t center, vec3_t half_extents);
/* A cylinder along the local X axis (a wheel), centred at center. */
bool mesh_data_add_cylinder_x(mesh_data_t* mesh, vec3_t center, float radius,
                              float width, int segments);
bool mesh_data_add_grid(mesh_data_t* mesh, float half_size, float spacing);
bool mesh_data_append(mesh_data_t* mesh, const mesh_data_t* other);
aabb_t mesh_data_bounds(const mesh_data_t* mesh);
size_t mesh_data_triangle_count(const mesh_data_t* mesh);

#endif  // ENGINE_RENDERER_MESH_DATA_H_
