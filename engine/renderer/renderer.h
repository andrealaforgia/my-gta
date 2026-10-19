/**
 * @file renderer.h
 * @brief The engine-facing rendering interface. A modest forward renderer:
 * static meshes, flat or textured metallic/roughness materials, one
 * directional light with ambient, distance fog, debug lines and text.
 * Nothing above this header sees OpenGL.
 */
#ifndef ENGINE_RENDERER_RENDERER_H_
#define ENGINE_RENDERER_RENDERER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mat4.h"
#include "mesh_data.h"
#include "vec3.h"

typedef struct renderer renderer_t;

typedef struct {
  uint32_t id; /* 0 is "none" */
} mesh_handle_t;

typedef struct {
  uint32_t id; /* 0 is "none" */
} texture_handle_t;

typedef struct {
  vec3_t base_color;
  float metallic;
  float roughness;
  texture_handle_t texture;
} material_t;

typedef struct {
  vec3_t eye;
  vec3_t target;
  vec3_t up;
  float fov_y;
  float near_plane;
  float far_plane;
} render_camera_t;

typedef struct {
  vec3_t direction; /* towards the light */
  vec3_t color;
  float ambient;
  vec3_t sky_color;
  float fog_start;
  float fog_end;
} render_environment_t;

typedef struct {
  int draw_calls;
  int triangles;
  int meshes_uploaded;
} render_stats_t;

material_t material_plain(vec3_t base_color);
material_t material_rough(vec3_t base_color, float roughness);

renderer_t* renderer_create(int drawable_width, int drawable_height);
void renderer_destroy(renderer_t* renderer);
void renderer_resize(renderer_t* renderer, int drawable_width, int drawable_height);

mesh_handle_t renderer_upload_mesh(renderer_t* renderer, const mesh_data_t* mesh);
void renderer_release_mesh(renderer_t* renderer, mesh_handle_t mesh);
/* RGBA8, tightly packed, repeats with mipmaps. */
texture_handle_t renderer_upload_texture(renderer_t* renderer, int width, int height,
                                         const uint8_t* rgba);
void renderer_release_texture(renderer_t* renderer, texture_handle_t texture);

void renderer_begin_frame(renderer_t* renderer, render_camera_t camera,
                          render_environment_t environment);
void renderer_draw_mesh(renderer_t* renderer, mesh_handle_t mesh, material_t material,
                        mat4_t model);
void renderer_draw_line(renderer_t* renderer, vec3_t a, vec3_t b, vec3_t color);
void renderer_draw_box_outline(renderer_t* renderer, mat4_t model, vec3_t half_extents,
                               vec3_t color);
/* Screen-space text in a built-in 5x7 font; x, y in pixels from top-left. */
void renderer_draw_text(renderer_t* renderer, float x, float y, float scale, vec3_t color,
                        const char* text);
void renderer_end_frame(renderer_t* renderer);

mat4_t renderer_view_projection(const renderer_t* renderer);
render_stats_t renderer_stats(const renderer_t* renderer);
/* RGBA8, bottom row first, drawable size. */
bool renderer_read_pixels(renderer_t* renderer, uint8_t* rgba, size_t size);

#endif  // ENGINE_RENDERER_RENDERER_H_
