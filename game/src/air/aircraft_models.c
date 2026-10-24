#include "aircraft_models.h"

#include <math.h>

/* Eight corners numbered as a box's are (bit 0 +x, bit 1 up, bit 2 +z),
 * joined into a solid: faces anticlockwise from outside. */
static void prism(mesh_data_t* m, const vec3_t p[8]) {
  mesh_data_add_quad(m, p[4], p[5], p[7], p[6], 1);
  mesh_data_add_quad(m, p[1], p[0], p[2], p[3], 1);
  mesh_data_add_quad(m, p[5], p[1], p[3], p[7], 1);
  mesh_data_add_quad(m, p[0], p[4], p[6], p[2], 1);
  mesh_data_add_quad(m, p[2], p[6], p[7], p[3], 1);
  mesh_data_add_quad(m, p[0], p[1], p[5], p[4], 1);
}

/* A swept wing from x0 to x1 (x0 < x1): at each end the leading and trailing
 * edge (z) and the height of the middle of the section; `thick` is the
 * depth. */
static void slab(mesh_data_t* m, float x0, float zf0, float zb0, float y0,
                 float x1, float zf1, float zb1, float y1, float thick) {
  vec3_t p[8];
  for (int i = 0; i < 8; i++) {
    bool far = i & 1, up = i & 2, back = i & 4;
    float y = (far ? y1 : y0) + (up ? thick : -thick) * 0.5f;
    float z = far ? (back ? zb1 : zf1) : (back ? zb0 : zf0);
    p[i] = vec3(far ? x1 : x0, y, z);
  }
  prism(m, p);
}

/* A fin standing on the centre line, swept: the foot's and the tip's edges. */
static void fin(mesh_data_t* m, float half, float y0, float zf0, float zb0,
                float y1, float zf1, float zb1) {
  vec3_t p[8];
  for (int i = 0; i < 8; i++) {
    bool right = i & 1, top = i & 2, back = i & 4;
    float z = top ? (back ? zb1 : zf1) : (back ? zb0 : zf0);
    p[i] = vec3(right ? half : -half, top ? y1 : y0, z);
  }
  prism(m, p);
}

/* Both sides of a swept wing, root at |x| = xr and tip at |x| = xt. */
static void wing_pair(mesh_data_t* m, float xr, float zfr, float zbr, float yr,
                      float xt, float zft, float zbt, float yt, float thick) {
  slab(m, xr, zfr, zbr, yr, xt, zft, zbt, yt, thick);
  slab(m, -xt, zft, zbt, yt, -xr, zfr, zbr, yr, thick);
}

void build_airliner(mesh_data_t* paint, mesh_data_t* trim, mesh_data_t* glass,
                    mesh_data_t* accent) {
  /* Fuselage, the hump of the upper deck, the tail cone and the fin. */
  mesh_data_add_ellipsoid(paint, vec3(0, 5.3f, 0), vec3(3.2f, 3.4f, 35.0f), 12,
                          22);
  mesh_data_add_ellipsoid(paint, vec3(0, 8.1f, -22.0f), vec3(2.3f, 1.7f, 10.0f),
                          8, 14);
  mesh_data_add_ellipsoid(paint, vec3(0, 6.9f, 28.0f), vec3(1.5f, 1.8f, 9.0f),
                          8, 14);
  fin(paint, 0.22f, 8.0f, 22.0f, 31.0f, 19.5f, 29.0f, 33.5f);
  /* Swept wings that rise a little, and the tail plane. */
  wing_pair(paint, 2.8f, -4.0f, 12.0f, 3.2f, 30.0f, 12.0f, 17.0f, 5.2f, 0.5f);
  wing_pair(paint, 1.5f, 26.0f, 31.0f, 7.2f, 12.5f, 31.0f, 34.0f, 8.0f, 0.2f);
  /* Four engines slung under the wings, ahead of them. */
  static const float engines[4][3] = {{-21.5f, 3.0f, 5.5f},
                                      {-11.5f, 2.4f, 0.0f},
                                      {11.5f, 2.4f, 0.0f},
                                      {21.5f, 3.0f, 5.5f}};
  for (int i = 0; i < 4; i++) {
    mesh_data_add_ellipsoid(paint,
                            vec3(engines[i][0], engines[i][1], engines[i][2]),
                            vec3(1.2f, 1.2f, 3.4f), 8, 12);
    mesh_data_add_ellipsoid(
        trim, vec3(engines[i][0], engines[i][1], engines[i][2] - 3.1f),
        vec3(0.95f, 0.95f, 0.5f), 6, 10);
    mesh_data_add_box(
        trim, vec3(engines[i][0], engines[i][1] + 1.3f, engines[i][2] + 0.5f),
        vec3(0.12f, 0.5f, 1.8f));
  }
  /* Landing gear: the nose leg and wheels, the body and the wing bogies. */
  mesh_data_add_box(trim, vec3(0, 1.6f, -24.0f), vec3(0.12f, 1.6f, 0.12f));
  mesh_data_add_cylinder_x(trim, vec3(-0.45f, 0.6f, -24.0f), 0.6f, 0.35f, 10);
  mesh_data_add_cylinder_x(trim, vec3(0.45f, 0.6f, -24.0f), 0.6f, 0.35f, 10);
  static const float bogies[3][2] = {{-4.2f, 4.0f}, {4.2f, 4.0f}, {0, 9.0f}};
  for (int i = 0; i < 3; i++) {
    float x = bogies[i][0], z = bogies[i][1];
    mesh_data_add_box(trim, vec3(x, 2.6f, z), vec3(0.15f, 1.5f, 0.15f));
    for (int k = 0; k < 4; k++)
      mesh_data_add_cylinder_x(
          trim,
          vec3(x + ((k & 1) ? 0.5f : -0.5f), 0.6f, z + (k & 2 ? 1.1f : -1.1f)),
          0.6f, 0.4f, 10);
  }
  /* Flight deck windows, and a window to each row along both sides. */
  mesh_data_add_box(glass, vec3(0, 8.65f, -30.4f), vec3(1.05f, 0.28f, 0.3f));
  for (float z = -16.0f; z <= 22.0f; z += 2.6f) {
    float f = sqrtf(1.0f - (z / 35.0f) * (z / 35.0f));
    float x = 3.2f * f * 0.956f + 0.03f;
    mesh_data_add_box(glass, vec3(x, 6.3f, z), vec3(0.03f, 0.17f, 0.45f));
    mesh_data_add_box(glass, vec3(-x, 6.3f, z), vec3(0.03f, 0.17f, 0.45f));
  }
  /* Livery: a cheat line along each side, the fin and the engine rings. */
  for (float z = -24.0f; z <= 26.0f; z += 2.0f) {
    float f = sqrtf(1.0f - (z / 35.0f) * (z / 35.0f));
    float x = 3.2f * f * 0.98f + 0.02f;
    mesh_data_add_box(accent, vec3(x, 5.5f, z), vec3(0.03f, 0.2f, 1.05f));
    mesh_data_add_box(accent, vec3(-x, 5.5f, z), vec3(0.03f, 0.2f, 1.05f));
  }
  fin(accent, 0.24f, 12.0f, 24.5f, 31.6f, 19.5f, 29.0f, 33.5f);
  for (int i = 0; i < 4; i++)
    mesh_data_add_ellipsoid(
        accent, vec3(engines[i][0], engines[i][1], engines[i][2] - 2.5f),
        vec3(1.25f, 1.25f, 0.35f), 6, 12);
}

void build_jet(mesh_data_t* paint, mesh_data_t* trim, mesh_data_t* glass,
               mesh_data_t* flame) {
  /* Fuselage, long nose, air intake under the cockpit. */
  mesh_data_add_ellipsoid(paint, vec3(0, 1.7f, 0), vec3(0.95f, 0.85f, 7.0f), 10,
                          16);
  mesh_data_add_ellipsoid(paint, vec3(0, 1.75f, -6.2f),
                          vec3(0.45f, 0.42f, 2.6f), 8, 12);
  mesh_data_add_box(trim, vec3(0, 1.0f, -2.0f), vec3(0.75f, 0.42f, 1.5f));
  /* Delta wings, the tail planes and the fin. */
  wing_pair(paint, 0.8f, -2.2f, 5.2f, 1.5f, 4.7f, 3.4f, 5.0f, 1.45f, 0.14f);
  wing_pair(paint, 0.8f, 4.6f, 6.9f, 1.7f, 3.0f, 6.0f, 7.1f, 1.75f, 0.1f);
  fin(paint, 0.07f, 2.3f, 3.4f, 7.0f, 4.4f, 5.6f, 7.1f);
  /* Engine nozzle, gear, and a missile rail under each wing. */
  mesh_data_add_ellipsoid(trim, vec3(0, 1.7f, 7.2f), vec3(0.72f, 0.7f, 0.6f), 6,
                          12);
  mesh_data_add_box(trim, vec3(0, 0.8f, -4.2f), vec3(0.04f, 0.5f, 0.04f));
  mesh_data_add_cylinder_x(trim, vec3(0, 0.28f, -4.2f), 0.28f, 0.16f, 10);
  for (int side = -1; side <= 1; side += 2) {
    mesh_data_add_box(trim, vec3(side * 1.4f, 0.8f, 1.8f),
                      vec3(0.04f, 0.5f, 0.05f));
    mesh_data_add_cylinder_x(trim, vec3(side * 1.4f, 0.3f, 1.8f), 0.3f, 0.2f,
                             10);
    for (int k = 0; k < 2; k++)
      mesh_data_add_box(trim,
                        vec3(side * (1.9f + 1.3f * (float)k), 1.38f, 1.2f),
                        vec3(0.02f, 0.06f, 0.5f));
  }
  mesh_data_add_ellipsoid(glass, vec3(0, 2.25f, -3.0f), vec3(0.5f, 0.5f, 1.5f),
                          8, 12);
  /* The flame: a long cone behind the nozzle. */
  mesh_data_add_ellipsoid(flame, vec3(0, 1.7f, 9.0f), vec3(0.42f, 0.42f, 1.9f),
                          6, 10);
}

void build_gunship(mesh_data_t* paint, mesh_data_t* trim, mesh_data_t* glass,
                   mesh_data_t* rotor) {
  /* A long body with a glazed nose, a pylon for each rotor and a ramp. */
  mesh_data_add_ellipsoid(paint, vec3(0, 2.3f, 0.3f), vec3(1.45f, 1.5f, 7.2f),
                          10, 16);
  mesh_data_add_ellipsoid(paint, vec3(0, 3.5f, -4.0f), vec3(0.7f, 0.8f, 1.2f),
                          6, 10);
  mesh_data_add_ellipsoid(paint, vec3(0, 3.8f, 5.6f), vec3(0.85f, 1.7f, 2.2f),
                          8, 12);
  mesh_data_add_box(paint, vec3(0, 1.95f, 7.4f), vec3(1.0f, 0.3f, 1.0f));
  mesh_data_add_ellipsoid(glass, vec3(0, 2.9f, -6.0f),
                          vec3(1.05f, 0.85f, 1.35f), 8, 12);
  /* Sponsons with the main wheels, and stub wings that carry the missiles. */
  for (int side = -1; side <= 1; side += 2) {
    mesh_data_add_ellipsoid(paint, vec3(side * 1.55f, 1.2f, 1.3f),
                            vec3(0.6f, 0.6f, 2.4f), 6, 10);
    mesh_data_add_cylinder_x(trim, vec3(side * 1.55f, 0.4f, 2.8f), 0.4f, 0.28f,
                             10);
    mesh_data_add_cylinder_x(trim, vec3(side * 1.55f, 0.4f, -1.4f), 0.4f, 0.28f,
                             10);
    mesh_data_add_box(paint, vec3(side * 2.8f, 2.0f, -1.8f),
                      vec3(1.5f, 0.06f, 0.55f));
    for (int k = 0; k < 2; k++)
      mesh_data_add_box(trim,
                        vec3(side * (2.3f + 1.0f * (float)k), 1.82f, -1.8f),
                        vec3(0.02f, 0.1f, 0.5f));
  }
  /* The gun under the chin, and the masts for the two rotors. */
  mesh_data_add_ellipsoid(trim, vec3(0, 1.1f, -5.6f), vec3(0.4f, 0.35f, 0.6f),
                          6, 10);
  mesh_data_add_box(trim, vec3(0, 1.0f, -6.7f), vec3(0.05f, 0.05f, 0.7f));
  mesh_data_add_cylinder_y(trim, vec3(0, 4.3f, -4.0f), 0.1f, 0.4f, 8);
  mesh_data_add_cylinder_y(trim, vec3(0, 5.5f, 5.6f), 0.1f, 0.45f, 8);
  /* One rotor: two blades crossed, drawn turned about each mast. */
  mesh_data_add_box(rotor, vec3(0, 0, 0), vec3(6.2f, 0.02f, 0.17f));
  mesh_data_add_box(rotor, vec3(0, 0, 0), vec3(0.17f, 0.02f, 6.2f));
}

void build_missile(mesh_data_t* body, mesh_data_t* flame) {
  mesh_data_add_ellipsoid(body, vec3(0, 0, 0), vec3(0.11f, 0.11f, 1.3f), 6, 10);
  mesh_data_add_box(body, vec3(0, 0, 0.8f), vec3(0.45f, 0.015f, 0.2f));
  mesh_data_add_box(body, vec3(0, 0, 0.8f), vec3(0.015f, 0.45f, 0.2f));
  mesh_data_add_ellipsoid(flame, vec3(0, 0, 1.9f), vec3(0.09f, 0.09f, 0.7f), 5,
                          8);
}
