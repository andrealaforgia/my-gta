/* A renderer for tests: records calls, draws nothing, needs no GPU. */
#include <stdlib.h>
#include <string.h>

#include "renderer.h"

struct renderer {
  mat4_t view_projection;
  render_stats_t stats;
  uint32_t next_mesh;
  uint32_t next_texture;
};

renderer_t* renderer_create(int drawable_width, int drawable_height) {
  (void)drawable_width;
  (void)drawable_height;
  renderer_t* r = calloc(1, sizeof(renderer_t));
  r->view_projection = mat4_identity();
  r->next_mesh = r->next_texture = 1;
  return r;
}

void renderer_destroy(renderer_t* r) { free(r); }
void renderer_resize(renderer_t* r, int w, int h) {
  (void)r;
  (void)w;
  (void)h;
}

mesh_handle_t renderer_upload_mesh(renderer_t* r, const mesh_data_t* mesh) {
  mesh_handle_t handle = {mesh->vertex_count ? r->next_mesh++ : 0};
  r->stats.meshes_uploaded++;
  return handle;
}

void renderer_release_mesh(renderer_t* r, mesh_handle_t mesh) {
  (void)mesh;
  r->stats.meshes_uploaded--;
}

texture_handle_t renderer_upload_texture(renderer_t* r, int w, int h,
                                         const uint8_t* rgba) {
  (void)w;
  (void)h;
  (void)rgba;
  texture_handle_t handle = {r->next_texture++};
  return handle;
}

void renderer_release_texture(renderer_t* r, texture_handle_t t) {
  (void)r;
  (void)t;
}

void renderer_begin_frame(renderer_t* r, render_camera_t camera,
                          render_environment_t env) {
  (void)env;
  mat4_t view = mat4_look_at(camera.eye, camera.target, camera.up);
  mat4_t projection = mat4_perspective(camera.fov_y, 16.0f / 9.0f,
                                       camera.near_plane, camera.far_plane);
  r->view_projection = mat4_multiply(projection, view);
  int uploaded = r->stats.meshes_uploaded;
  memset(&r->stats, 0, sizeof(r->stats));
  r->stats.meshes_uploaded = uploaded;
}

void renderer_draw_mesh(renderer_t* r, mesh_handle_t mesh, material_t material,
                        mat4_t model) {
  (void)material;
  (void)model;
  if (mesh.id) r->stats.draw_calls++;
}

void renderer_draw_line(renderer_t* r, vec3_t a, vec3_t b, vec3_t c) {
  (void)r;
  (void)a;
  (void)b;
  (void)c;
}
void renderer_draw_box_outline(renderer_t* r, mat4_t m, vec3_t h, vec3_t c) {
  (void)r;
  (void)m;
  (void)h;
  (void)c;
}
void renderer_draw_text(renderer_t* r, float x, float y, float s, vec3_t c,
                        const char* t) {
  (void)r;
  (void)x;
  (void)y;
  (void)s;
  (void)c;
  (void)t;
}
void renderer_end_frame(renderer_t* r) { (void)r; }
mat4_t renderer_view_projection(const renderer_t* r) {
  return r->view_projection;
}
render_stats_t renderer_stats(const renderer_t* r) { return r->stats; }
int renderer_read_shadow_map(renderer_t* r, float* d, size_t n) {
  (void)r;
  (void)d;
  (void)n;
  return 0;
}
float renderer_shadow_coverage(renderer_t* r) {
  (void)r;
  return 0;
}
bool renderer_read_pixels(renderer_t* r, uint8_t* rgba, size_t size) {
  (void)r;
  (void)rgba;
  (void)size;
  return false;
}
