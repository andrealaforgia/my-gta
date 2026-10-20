/* The pause-screen map: the city flattened to a picture, north up. */
#include <assert.h>
#include <math.h>
#include <string.h>

#include "city_map.h"
#include "city_package.h"
#include "test_groups.h"
#include "test_support.h"

static void add_square(city_chunk_t* chunk, const char* material, float x0,
                       float z0, float x1, float z1) {
  city_part_t* part =
      city_chunk_add_part(chunk, material, PART_COLLIDER_NONE, 0, 0, 1);
  assert(part);
  vec3_t up = vec3(0, 1, 0);
  vertex_t v[4] = {{vec3(x0, 5, z0), up, 0, 0},
                   {vec3(x1, 5, z0), up, 0, 0},
                   {vec3(x1, 5, z1), up, 0, 0},
                   {vec3(x0, 5, z1), up, 0, 0}};
  for (int i = 0; i < 4; i++) assert(mesh_data_add_vertex(&part->mesh, v[i]));
  assert(mesh_data_add_triangle(&part->mesh, 0, 1, 2));
  assert(mesh_data_add_triangle(&part->mesh, 0, 2, 3));
  chunk->bounds = aabb_merge(chunk->bounds, mesh_data_bounds(&part->mesh));
}

static city_map_t map_of_two_squares(void) {
  city_package_t package = city_package_create();
  city_chunk_t* chunk = city_package_chunk(&package, 0, 0);
  /* A road across the middle and a roof north-west of it. */
  add_square(chunk, "road", -100, -5, 100, 5);
  add_square(chunk, "roof", -80, -60, -40, -20);
  city_map_t map;
  assert(city_map_build(&map, &package, 256));
  city_package_free(&package);
  return map;
}

static uint32_t pixel_at(const city_map_t* map, float x, float z) {
  int px, py;
  assert(city_map_to_pixel(map, x, z, &px, &py));
  const uint8_t* p = map->pixels + ((size_t)py * map->size + px) * 4;
  return (uint32_t)p[0] << 16 | (uint32_t)p[1] << 8 | p[2];
}

static void roads_and_roofs_are_drawn_where_they_are_north_up(void) {
  city_map_t map = map_of_two_squares();
  uint32_t ground = pixel_at(&map, 90, -40);
  uint32_t road = pixel_at(&map, 0, 0);
  uint32_t roof = pixel_at(&map, -60, -40);
  assert(road != ground && roof != ground && road != roof);
  /* North (negative z) is up the picture: the roof is above the road. */
  int rx, ry, bx, by;
  assert(city_map_to_pixel(&map, -60, -40, &rx, &ry));
  assert(city_map_to_pixel(&map, 0, 0, &bx, &by));
  assert(ry < by);
  /* East is to the right. */
  int ex, ey;
  assert(city_map_to_pixel(&map, 50, 0, &ex, &ey));
  assert(ex > bx && ey == by);
  city_map_free(&map);
}

static void points_outside_the_city_are_not_on_the_map(void) {
  city_map_t map = map_of_two_squares();
  int px, py;
  assert(!city_map_to_pixel(&map, 5000, 0, &px, &py));
  assert(!city_map_to_pixel(&map, 0, -5000, &px, &py));
  city_map_free(&map);
}

static void the_minimap_window_follows_the_car_and_stays_on_the_map(void) {
  city_map_t map = map_of_two_squares();
  float u0, v0, u1, v1;
  /* Centred on the middle of the city: a square 100 m across. */
  city_map_window(&map, 0.0f, -27.5f, 50.0f, &u0, &v0, &u1, &v1);
  float span = 100.0f / map.extent;
  assert(near_to(u1 - u0, span, 1e-4f) && near_to(v1 - v0, span, 1e-4f));
  assert(near_to(0.5f * (u0 + u1), 0.5f, 0.01f));
  /* Beyond an edge it slides back so the window is still full size. */
  city_map_window(&map, 5000.0f, 5000.0f, 50.0f, &u0, &v0, &u1, &v1);
  assert(u1 <= 1.0001f && v1 <= 1.0001f && near_to(u1 - u0, span, 1e-4f));
  city_map_window(&map, -5000.0f, -5000.0f, 50.0f, &u0, &v0, &u1, &v1);
  assert(u0 >= -0.0001f && v0 >= -0.0001f);
  /* A window larger than the city shows all of it. */
  city_map_window(&map, 0.0f, 0.0f, 5000.0f, &u0, &v0, &u1, &v1);
  assert(near_to(u0, 0.0f, 1e-4f) && near_to(u1, 1.0f, 1e-4f));
  city_map_free(&map);
}

void run_citymap_tests(void) {
  RUN(the_minimap_window_follows_the_car_and_stays_on_the_map);
  RUN(roads_and_roofs_are_drawn_where_they_are_north_up);
  RUN(points_outside_the_city_are_not_on_the_map);
}
