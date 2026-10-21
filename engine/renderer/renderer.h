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
  float emissive; /* 0..: the colour added unlit, for lamps */
  /* 0..1: how many of the panes a texture marks as glass (alpha 0) are lit
   * from inside, chosen per window by position. */
  float window_glow;
  /* 0..1: darkens a wall towards its foot, as grime and contact shadow do.
   * Reads the height up the wall from the texture's v (storeys). */
  float wall_ao;
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
  float shadow_radius; /* metres around the camera target; 0 for no shadows */
  /* The sky behind everything: sky_color is the horizon (and the fog), this
   * is straight up; star_level 0..1 fades stars in; sun_disc 0..1 draws
   * the sun in the direction of the light. */
  vec3_t zenith_color;
  float star_level;
  float sun_disc;
  /* One headlight cone for the player's car: position, the way it points,
   * and how strong; 0 is off. */
  vec3_t spot_position;
  vec3_t spot_direction;
  float spot_intensity;
  /* Clouds in the sky: how much of it they cover (0 clear, 1 overcast) and
   * a clock in seconds that carries them along on the wind. */
  float cloud_cover;
  float cloud_time;
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
void renderer_resize(renderer_t* renderer, int drawable_width,
                     int drawable_height);

mesh_handle_t renderer_upload_mesh(renderer_t* renderer,
                                   const mesh_data_t* mesh);
void renderer_release_mesh(renderer_t* renderer, mesh_handle_t mesh);
/* RGBA8, tightly packed, repeats with mipmaps. */
texture_handle_t renderer_upload_texture(renderer_t* renderer, int width,
                                         int height, const uint8_t* rgba);
void renderer_release_texture(renderer_t* renderer, texture_handle_t texture);

void renderer_begin_frame(renderer_t* renderer, render_camera_t camera,
                          render_environment_t environment);
void renderer_draw_mesh(renderer_t* renderer, mesh_handle_t mesh,
                        material_t material, mat4_t model);
void renderer_draw_line(renderer_t* renderer, vec3_t a, vec3_t b, vec3_t color);
/* Like renderer_draw_line but hidden behind solid things, for weather. */
void renderer_draw_line_tested(renderer_t* renderer, vec3_t a, vec3_t b,
                               vec3_t color);
void renderer_draw_box_outline(renderer_t* renderer, mat4_t model,
                               vec3_t half_extents, vec3_t color);
/* Screen-space text in a built-in 5x7 font; x, y in pixels from top-left. */
void renderer_draw_text(renderer_t* renderer, float x, float y, float scale,
                        vec3_t color, const char* text);
/* A textured rectangle in pixels from the top-left, tinted and blended with
 * the given alpha over the scene and under the text. A texture handle of id
 * 0 draws a plain tinted rectangle. At most 8 per frame. */
void renderer_draw_screen_image(renderer_t* renderer, texture_handle_t texture,
                                float x, float y, float width, float height,
                                vec3_t tint, float alpha);
/* The same, showing only the part of the texture from (u0, v0) to (u1, v1). */
void renderer_draw_screen_image_uv(renderer_t* renderer,
                                   texture_handle_t texture, float x, float y,
                                   float width, float height, float u0,
                                   float v0, float u1, float v1, vec3_t tint,
                                   float alpha);
/* A solid line of the given thickness in pixels, drawn with the text. */
void renderer_draw_screen_line(renderer_t* renderer, float x0, float y0,
                               float x1, float y1, float thickness,
                               vec3_t color);
void renderer_end_frame(renderer_t* renderer);

mat4_t renderer_view_projection(const renderer_t* renderer);
render_stats_t renderer_stats(const renderer_t* renderer);
/* Fraction of the shadow map written last frame, for diagnostics. */
float renderer_shadow_coverage(renderer_t* renderer);
/* Copies the shadow map depths (size x size floats); returns the size, 0 if
 * none. */
int renderer_read_shadow_map(renderer_t* renderer, float* depths, size_t count);
/* RGBA8, bottom row first, drawable size. */
bool renderer_read_pixels(renderer_t* renderer, uint8_t* rgba, size_t size);

#endif  // ENGINE_RENDERER_RENDERER_H_
