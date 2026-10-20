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
  RUN(the_font_covers_digits_and_capitals);
}
