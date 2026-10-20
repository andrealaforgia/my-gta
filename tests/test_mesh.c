#include <assert.h>

#include "font5x7.h"
#include "mesh_data.h"
#include "test_groups.h"
#include "test_support.h"

static void a_box_has_twelve_outward_facing_triangles(void) {
  mesh_data_t mesh = mesh_data_create();
  assert(mesh_data_add_box(&mesh, vec3(0, 0, 0), vec3(1, 2, 3)));
  assert(mesh_data_triangle_count(&mesh) == 12);
  for (size_t i = 0; i < mesh.index_count; i += 3) {
    vec3_t a = mesh.vertices[mesh.indices[i]].position;
    vec3_t b = mesh.vertices[mesh.indices[i + 1]].position;
    vec3_t c = mesh.vertices[mesh.indices[i + 2]].position;
    vec3_t face = vec3_cross(vec3_sub(b, a), vec3_sub(c, a));
    vec3_t centre = vec3_scale(vec3_add(vec3_add(a, b), c), 1.0f / 3.0f);
    assert(vec3_dot(face, centre) > 0); /* anticlockwise seen from outside */
    assert(vec3_dot(mesh.vertices[mesh.indices[i]].normal, centre) > 0);
  }
  aabb_t bounds = mesh_data_bounds(&mesh);
  assert(near(bounds.min.y, -2) && near(bounds.max.z, 3));
  mesh_data_free(&mesh);
}

static void a_wheel_cylinder_spans_its_width_and_radius(void) {
  mesh_data_t mesh = mesh_data_create();
  assert(mesh_data_add_cylinder_x(&mesh, vec3(1, 0, 0), 0.3f, 0.2f, 16));
  aabb_t bounds = mesh_data_bounds(&mesh);
  assert(near(bounds.min.x, 0.9f) && near(bounds.max.x, 1.1f));
  assert(near(bounds.max.y, 0.3f) && near(bounds.min.z, -0.3f));
  assert(mesh_data_triangle_count(&mesh) == 16 * 2 + 16 * 2);
  mesh_data_free(&mesh);
}

static void meshes_append_with_rebased_indices(void) {
  mesh_data_t a = mesh_data_create();
  mesh_data_t b = mesh_data_create();
  mesh_data_add_box(&a, vec3(0, 0, 0), vec3(1, 1, 1));
  mesh_data_add_box(&b, vec3(5, 0, 0), vec3(1, 1, 1));
  assert(mesh_data_append(&a, &b));
  assert(mesh_data_triangle_count(&a) == 24);
  assert(a.indices[a.index_count - 1] >= 24);
  mesh_data_free(&a);
  mesh_data_free(&b);
}

static float triangle_area(vec3_t a, vec3_t b, vec3_t c) {
  return 0.5f * vec3_length(vec3_cross(vec3_sub(b, a), vec3_sub(c, a)));
}

static void a_large_triangle_is_subdivided_until_its_edges_are_short(void) {
  mesh_data_t mesh = mesh_data_create();
  vertex_t a = {vec3(0, 0, 0), vec3(0, 1, 0), 0, 0};
  vertex_t b = {vec3(40, 0, 0), vec3(0, 1, 0), 4, 0};
  vertex_t c = {vec3(0, 0, 30), vec3(0, 1, 0), 0, 3};
  assert(mesh_data_add_vertex(&mesh, a) && mesh_data_add_vertex(&mesh, b) &&
         mesh_data_add_vertex(&mesh, c));
  assert(mesh_data_add_triangle(&mesh, 0, 2, 1));
  assert(mesh_data_subdivide(&mesh, 10.0f));
  assert(mesh_data_triangle_count(&mesh) > 8);
  float area = 0;
  for (size_t i = 0; i + 2 < mesh.index_count; i += 3) {
    vec3_t p = mesh.vertices[mesh.indices[i]].position;
    vec3_t q = mesh.vertices[mesh.indices[i + 1]].position;
    vec3_t r = mesh.vertices[mesh.indices[i + 2]].position;
    assert(vec3_distance(p, q) <= 10.001f && vec3_distance(q, r) <= 10.001f &&
           vec3_distance(r, p) <= 10.001f);
    area += triangle_area(p, q, r);
    /* The winding is kept: same facing as the original (0, 2, 1). */
    assert(vec3_cross(vec3_sub(q, p), vec3_sub(r, p)).y > 0);
  }
  assert(near_to(area, 600.0f, 0.01f)); /* 40 x 30 / 2 */
  /* Texture coordinates follow the positions (u = x / 10, v = z / 10). */
  for (size_t i = 0; i < mesh.vertex_count; i++) {
    assert(near_to(mesh.vertices[i].u, mesh.vertices[i].position.x / 10.0f,
                   1e-4f));
    assert(near_to(mesh.vertices[i].v, mesh.vertices[i].position.z / 10.0f,
                   1e-4f));
  }
  mesh_data_free(&mesh);
}

static void the_font_covers_digits_and_capitals(void) {
  for (char c = '0'; c <= '9'; c++) assert(font5x7_has_glyph(c));
  for (char c = 'A'; c <= 'Z'; c++) assert(font5x7_has_glyph(c));
  assert(font5x7_has_glyph('a'));
  assert(!font5x7_has_glyph('~'));
  assert(font5x7_pixel('I', 0, 1) && font5x7_pixel('I', 3, 2) &&
         !font5x7_pixel('I', 3, 0));
}

void run_mesh_tests(void) {
  RUN(a_box_has_twelve_outward_facing_triangles);
  RUN(a_wheel_cylinder_spans_its_width_and_radius);
  RUN(meshes_append_with_rebased_indices);
  RUN(a_large_triangle_is_subdivided_until_its_edges_are_short);
  RUN(the_font_covers_digits_and_capitals);
}
