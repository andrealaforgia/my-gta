/* Parked and moving cars: the fleet, the cars at the kerb, and traffic that
 * follows the roads, turns at junctions, keeps its distance and stops. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fleet.h"
#include "parked_cars.h"
#include "test_groups.h"
#include "test_support.h"
#include "traffic.h"

static physics_world_t* ground_world(void) {
  physics_world_t* world = physics_world_create();
  vec3_t v[4] = {vec3(-500, 2, 500), vec3(500, 2, 500), vec3(500, 2, -500),
                 vec3(-500, 2, -500)};
  uint32_t i[6] = {0, 1, 2, 0, 2, 3};
  physics_add_static_mesh(world, v, 4, i, 6, COLLISION_LAYER_GROUND, 1.0f, 0);
  return world;
}

static void the_fleet_has_four_bodies_and_a_palette(void) {
  renderer_t* renderer = renderer_create(64, 64);
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  float length[FLEET_VARIANTS];
  for (int i = 0; i < FLEET_VARIANTS; i++)
    length[i] = fleet.visuals[i].half_extents.z;
  /* hatchback 0, saloon 1, van 2, city car 3: the saloon is longer than the
   * hatchback, the city car shorter, the van tallest. */
  assert(length[1] > length[0] && length[3] < length[0]);
  assert(fleet.visuals[2].half_extents.y > fleet.visuals[0].half_extents.y);
  for (int i = 0; i < FLEET_VARIANTS; i++)
    assert(fleet.visuals[i].ride_height > 0.3f);
  int variants_seen[FLEET_VARIANTS] = {0};
  for (int pick = 0; pick < 100; pick++)
    variants_seen[fleet_variant_for(pick)]++;
  for (int i = 0; i < FLEET_VARIANTS; i++) assert(variants_seen[i] > 0);
  assert(vec3_distance(fleet_paint(0), fleet_paint(7)) > 0.05f);
  fleet_destroy(&fleet, renderer);
  renderer_destroy(renderer);
}

static void parked_cars_stand_on_the_ground_and_are_solid(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_parking_t spots[3] = {{0, 0, 0, 3}, {10, 0, 0.5f, 5}, {800, 0, 0, 1}};
  size_t before = physics_collider_count(world);
  parked_cars_t parked;
  assert(parked_cars_create(&parked, spots, 3, world, &fleet));
  assert(parked.count == 2); /* the third has no ground under it */
  assert(physics_collider_count(world) == before + 2);
  /* The car stands on the ground (at height 2), body above the road. */
  assert(parked.cars[0].position.y > 2.3f && parked.cars[0].position.y < 3.0f);
  ray_t from_above = {vec3(0, 20, 0), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  assert(physics_ray_cast(world, from_above, 50, COLLISION_LAYER_PROP, &hit));
  /* Only the ones near the eye are drawn. */
  int near_drawn =
      parked_cars_draw(&parked, &fleet, renderer, vec3(0, 1, 5), 50, 0);
  int far_drawn =
      parked_cars_draw(&parked, &fleet, renderer, vec3(400, 1, 0), 50, 0);
  assert(near_drawn == 2 && far_drawn == 0);
  parked_cars_destroy(&parked, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

/* Three roads meeting at (0,0): A west to the junction, B east, C south. */
static float west[4] = {-100, 0, 0, 0}, east[4] = {0, 0, 100, 0},
             south[4] = {0, 0, 0, 100};

static city_route_t route(float* points, int64_t start, int64_t end,
                          int oneway) {
  city_route_t r;
  memset(&r, 0, sizeof(r));
  r.drivable = 1;
  r.oneway = oneway;
  r.width = 6.0f;
  r.pavement = 2.0f;
  r.start_node = start;
  r.end_node = end;
  r.points = points;
  r.point_count = 2;
  return r;
}

static void cars_follow_the_roads_on_the_right_at_a_sensible_speed(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  traffic_t* traffic = traffic_create(routes, 3, world, &fleet, 6, 42);
  assert(traffic && traffic_car_count(traffic) == 6);
  for (int step = 0; step < 120 * 20; step++)
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
  for (size_t i = 0; i < traffic_car_count(traffic); i++) {
    traffic_car_t car = traffic_car(traffic, i);
    assert(car.speed >= 0 && car.speed < 13.0f);
    /* On one of the three centre lines, to its right: 1 to 2 m off it. */
    double best = 1e9;
    for (int r = 0; r < 3; r++) {
      const float* p = routes[r].points;
      double dx = p[2] - p[0], dz = p[3] - p[1], l2 = dx * dx + dz * dz;
      double u =
          ((car.position.x - p[0]) * dx + (car.position.z - p[1]) * dz) / l2;
      u = u < -0.02 ? -0.02 : (u > 1.02 ? 1.02 : u);
      best = fmin(best, hypot(car.position.x - (p[0] + u * dx),
                              car.position.z - (p[1] + u * dz)));
    }
    assert(best > 0.8 && best < 2.2);
  }
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void cars_turn_at_junctions_onto_every_road_and_keep_apart(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  traffic_t* traffic = traffic_create(routes, 3, world, &fleet, 8, 7);
  int visited[3] = {0, 0, 0};
  float closest = 1e9f;
  for (int step = 0; step < 120 * 240; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    if (step % 30) continue;
    for (size_t i = 0; i < traffic_car_count(traffic); i++) {
      traffic_car_t a = traffic_car(traffic, i);
      visited[a.route]++;
      for (size_t j = i + 1; j < traffic_car_count(traffic); j++)
        closest =
            fminf(closest,
                  vec3_distance(a.position, traffic_car(traffic, j).position));
    }
  }
  assert(visited[0] > 0 && visited[1] > 0 && visited[2] > 0);
  assert(closest > 2.4f); /* opposing lanes pass 3 m apart, never closer */
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void a_car_stops_short_of_the_player_in_its_way(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)}; /* one way, east */
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 3);
  traffic_car_place(traffic, 0, 0, 1, 10.0f);
  /* The player stands in the car's lane at x = -50, on the right of travel
   * (south of the centre line for an eastbound car). */
  vec3_t player = vec3(-50, 2.7f, 1.5f);
  float closest = 1e9f;
  for (int step = 0; step < 120 * 30; step++) {
    traffic_step(traffic, world, player, vec3_zero(), 1.0f / 120.0f);
    closest =
        fminf(closest, vec3_distance(traffic_car(traffic, 0).position, player));
  }
  traffic_car_t car = traffic_car(traffic, 0);
  assert(closest > 3.0f);
  assert(car.speed < 0.3f);
  assert(car.position.x > -65.0f &&
         car.position.x < -52.0f); /* came right up to it */
  /* Once the player moves off, it carries on (and, the road being a
   * one-way dead end, starts over from the beginning). */
  float furthest = -1e9f;
  for (int step = 0; step < 120 * 10; step++) {
    traffic_step(traffic, world, vec3(0, 2.7f, 60), vec3_zero(), 1.0f / 120.0f);
    furthest = fmaxf(furthest, traffic_car(traffic, 0).position.x);
  }
  assert(furthest > -20.0f);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void at_a_dead_end_a_car_turns_round_and_comes_back(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 11);
  traffic_car_place(traffic, 0, 0, 1, 5.0f);
  float most = -1e9f, least = 1e9f;
  for (int step = 0; step < 120 * 90; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    float x = traffic_car(traffic, 0).position.x;
    most = fmaxf(most, x), least = fminf(least, x);
  }
  assert(most > -8.0f);   /* reached the far end */
  assert(least < -60.0f); /* and came back along the road */
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void each_car_carries_a_solid_box_that_moves_with_it(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)};
  size_t before = physics_collider_count(world);
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 5);
  assert(physics_collider_count(world) == before + 1);
  traffic_car_place(traffic, 0, 0, 1, 40.0f);
  traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(), 1.0f / 120.0f);
  traffic_car_t car = traffic_car(traffic, 0);
  ray_t down = {vec3_add(car.position, vec3(0, 5, 0)), vec3(0, -1, 0)};
  ray_cast_result_t hit;
  assert(physics_ray_cast(world, down, 20, COLLISION_LAYER_PROP, &hit));
  for (int step = 0; step < 120 * 5; step++)
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
  car = traffic_car(traffic, 0);
  ray_t again = {vec3_add(car.position, vec3(0, 5, 0)), vec3(0, -1, 0)};
  assert(physics_ray_cast(world, again, 20, COLLISION_LAYER_PROP, &hit));
  assert(car.position.x > -55.0f); /* it moved on */
  traffic_destroy(traffic, world);
  assert(physics_collider_count(world) == before + 1 ||
         physics_collider_count(world) == before);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

void run_traffic_tests(void) {
  RUN(the_fleet_has_four_bodies_and_a_palette);
  RUN(parked_cars_stand_on_the_ground_and_are_solid);
  RUN(cars_follow_the_roads_on_the_right_at_a_sensible_speed);
  RUN(cars_turn_at_junctions_onto_every_road_and_keep_apart);
  RUN(a_car_stops_short_of_the_player_in_its_way);
  RUN(at_a_dead_end_a_car_turns_round_and_comes_back);
  RUN(each_car_carries_a_solid_box_that_moves_with_it);
}
