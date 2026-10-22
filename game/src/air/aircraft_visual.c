#include "aircraft_visual.h"

#include "mesh_data.h"

static void build_plane(mesh_data_t* paint, mesh_data_t* trim,
                        mesh_data_t* glass, mesh_data_t* prop) {
  /* Fuselage and tail cone, high wing, tail plane and fin. */
  mesh_data_add_ellipsoid(paint, vec3(0, 1.3f, 0), vec3(0.85f, 0.8f, 3.4f), 10,
                          16);
  mesh_data_add_ellipsoid(paint, vec3(0, 1.55f, 4.0f), vec3(0.34f, 0.4f, 2.4f),
                          8, 12);
  mesh_data_add_box(paint, vec3(0, 2.05f, -0.4f), vec3(5.6f, 0.07f, 0.95f));
  mesh_data_add_box(paint, vec3(0, 1.72f, 5.9f), vec3(1.9f, 0.05f, 0.5f));
  mesh_data_add_box(paint, vec3(0, 2.3f, 5.95f), vec3(0.05f, 0.7f, 0.55f));
  /* Struts from the fuselage up to the wing. */
  for (int side = -1; side <= 1; side += 2)
    mesh_data_add_box(trim, vec3(side * 1.6f, 1.6f, -0.4f),
                      vec3(0.03f, 0.55f, 0.05f));
  /* A stripe along each side, the wheels and their legs, the spinner. */
  for (int side = -1; side <= 1; side += 2) {
    mesh_data_add_box(trim, vec3(side * 0.855f, 1.2f, 0.2f),
                      vec3(0.012f, 0.09f, 2.6f));
    mesh_data_add_cylinder_x(trim, vec3(side * 1.05f, 0.3f, -0.8f), 0.3f, 0.16f,
                             10);
    mesh_data_add_box(trim, vec3(side * 0.95f, 0.75f, -0.8f),
                      vec3(0.03f, 0.5f, 0.04f));
  }
  mesh_data_add_cylinder_x(trim, vec3(0, 0.26f, -2.3f), 0.26f, 0.14f, 10);
  mesh_data_add_box(trim, vec3(0, 0.7f, -2.3f), vec3(0.03f, 0.5f, 0.04f));
  mesh_data_add_ellipsoid(trim, vec3(0, 1.3f, -3.75f), vec3(0.2f, 0.2f, 0.3f),
                          6, 10);
  /* The canopy, and the propeller: two blades, drawn turned. */
  mesh_data_add_ellipsoid(glass, vec3(0, 1.85f, -0.7f), vec3(0.66f, 0.5f, 1.2f),
                          8, 14);
  mesh_data_add_box(prop, vec3(0, 0, 0), vec3(0.05f, 1.05f, 0.025f));
}

static void build_helicopter(mesh_data_t* paint, mesh_data_t* trim,
                             mesh_data_t* glass, mesh_data_t* rotor,
                             mesh_data_t* tail_rotor) {
  mesh_data_add_ellipsoid(paint, vec3(0, 1.45f, -0.3f), vec3(1.0f, 0.95f, 1.9f),
                          10, 16);
  mesh_data_add_ellipsoid(paint, vec3(0, 2.4f, 0.5f), vec3(0.6f, 0.35f, 1.0f),
                          6, 10);
  mesh_data_add_box(paint, vec3(0, 1.75f, 3.3f), vec3(0.14f, 0.17f, 2.7f));
  mesh_data_add_box(paint, vec3(0, 2.3f, 5.7f), vec3(0.04f, 0.55f, 0.4f));
  mesh_data_add_box(paint, vec3(0, 1.95f, 5.4f), vec3(0.8f, 0.03f, 0.25f));
  /* Skids and the legs that hold the cabin on them. */
  for (int side = -1; side <= 1; side += 2) {
    mesh_data_add_box(trim, vec3(side * 0.95f, 0.1f, -0.2f),
                      vec3(0.04f, 0.04f, 1.8f));
    mesh_data_add_box(trim, vec3(side * 0.95f, 0.5f, -0.9f),
                      vec3(0.03f, 0.42f, 0.03f));
    mesh_data_add_box(trim, vec3(side * 0.95f, 0.5f, 0.6f),
                      vec3(0.03f, 0.42f, 0.03f));
  }
  mesh_data_add_cylinder_y(trim, vec3(0, 2.7f, 0.5f), 0.08f, 0.45f, 8);
  mesh_data_add_ellipsoid(glass, vec3(0, 1.62f, -1.25f),
                          vec3(0.9f, 0.8f, 0.95f), 8, 14);
  /* Two blades across the rotor head, drawn turned about the mast. */
  mesh_data_add_box(rotor, vec3(0, 0, 0), vec3(6.2f, 0.02f, 0.16f));
  mesh_data_add_box(rotor, vec3(0, 0, 0), vec3(0.16f, 0.02f, 6.2f));
  mesh_data_add_box(tail_rotor, vec3(0, 0, 0), vec3(0.03f, 0.55f, 0.07f));
  mesh_data_add_box(tail_rotor, vec3(0, 0, 0), vec3(0.03f, 0.07f, 0.55f));
}

static mesh_handle_t upload(renderer_t* renderer, mesh_data_t* mesh) {
  mesh_handle_t handle = renderer_upload_mesh(renderer, mesh);
  mesh_data_free(mesh);
  return handle;
}

aircraft_visual_t aircraft_visual_create(renderer_t* renderer) {
  aircraft_visual_t v;
  mesh_data_t a = mesh_data_create(), b = mesh_data_create(),
              c = mesh_data_create(), d = mesh_data_create();
  build_plane(&a, &b, &c, &d);
  v.plane_paint = upload(renderer, &a);
  v.plane_trim = upload(renderer, &b);
  v.plane_glass = upload(renderer, &c);
  v.plane_prop = upload(renderer, &d);
  a = mesh_data_create(), b = mesh_data_create(), c = mesh_data_create(),
  d = mesh_data_create();
  mesh_data_t e = mesh_data_create();
  build_helicopter(&a, &b, &c, &d, &e);
  v.heli_paint = upload(renderer, &a);
  v.heli_trim = upload(renderer, &b);
  v.heli_glass = upload(renderer, &c);
  v.heli_rotor = upload(renderer, &d);
  v.heli_tail_rotor = upload(renderer, &e);
  return v;
}

void aircraft_visual_destroy(aircraft_visual_t* v, renderer_t* renderer) {
  mesh_handle_t all[] = {v->plane_paint, v->plane_trim, v->plane_glass,
                         v->plane_prop,  v->heli_paint, v->heli_trim,
                         v->heli_glass,  v->heli_rotor, v->heli_tail_rotor};
  for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++)
    renderer_release_mesh(renderer, all[i]);
}

/* A part hung at `local` on the aircraft and turned about an axis. */
static mat4_t turned(transform_t base, vec3_t local, vec3_t axis, float angle) {
  transform_t part = base;
  part.position = transform_point(base, local);
  part.rotation =
      quat_multiply(base.rotation, quat_from_axis_angle(axis, angle));
  return transform_to_matrix(part);
}

void aircraft_visual_draw(const aircraft_visual_t* v, renderer_t* renderer,
                          const aircraft_t* a, int variant) {
  static const float paints[5][3] = {{0.92f, 0.92f, 0.9f},
                                     {0.85f, 0.15f, 0.12f},
                                     {0.95f, 0.78f, 0.15f},
                                     {0.15f, 0.3f, 0.7f},
                                     {0.85f, 0.85f, 0.88f}};
  const float* p = paints[((variant % 5) + 5) % 5];
  material_t paint = material_rough(vec3(p[0], p[1], p[2]), 0.35f);
  paint.metallic = 0.3f;
  material_t trim = material_rough(vec3(0.1f, 0.1f, 0.11f), 0.6f);
  material_t glass = material_rough(vec3(0.1f, 0.15f, 0.22f), 0.1f);
  glass.metallic = 0.6f;
  transform_t base = aircraft_transform(a);
  mat4_t model = transform_to_matrix(base);
  if (a->kind == AIRCRAFT_PLANE) {
    renderer_draw_mesh(renderer, v->plane_paint, paint, model);
    renderer_draw_mesh(renderer, v->plane_trim, trim, model);
    renderer_draw_mesh(renderer, v->plane_glass, glass, model);
    renderer_draw_mesh(
        renderer, v->plane_prop, trim,
        turned(base, vec3(0, 1.3f, -3.95f), vec3(0, 0, 1), a->rotor));
    return;
  }
  renderer_draw_mesh(renderer, v->heli_paint, paint, model);
  renderer_draw_mesh(renderer, v->heli_trim, trim, model);
  renderer_draw_mesh(renderer, v->heli_glass, glass, model);
  renderer_draw_mesh(
      renderer, v->heli_rotor, trim,
      turned(base, vec3(0, 3.15f, 0.5f), vec3(0, 1, 0), a->rotor));
  renderer_draw_mesh(
      renderer, v->heli_tail_rotor, trim,
      turned(base, vec3(0.12f, 2.3f, 5.7f), vec3(1, 0, 0), a->rotor * 2.5f));
}
