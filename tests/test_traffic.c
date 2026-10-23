/* Parked and moving cars: the fleet, the cars at the kerb, and traffic that
 * follows the roads, turns at junctions, keeps its distance and stops. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fleet.h"
#include "navigation.h"
#include "parked_cars.h"
#include "pedestrians.h"
#include "skid_marks.h"
#include "test_groups.h"
#include "test_support.h"
#include "traffic.h"
#include "walker.h"
#include "wanted.h"

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
  for (int i = 0; i < FLEET_CAR_VARIANTS; i++) assert(variants_seen[i] > 0);
  for (int i = FLEET_CAR_VARIANTS; i < FLEET_VARIANTS; i++)
    assert(variants_seen[i] == 0); /* parked cars are only cars */
  assert(vec3_distance(fleet_paint(0), fleet_paint(7)) > 0.05f);
  fleet_destroy(&fleet, renderer);
  renderer_destroy(renderer);
}

static void the_road_carries_buses_lorries_and_the_emergency_services(void) {
  renderer_t* renderer = renderer_create(64, 64);
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  int seen[FLEET_VARIANTS] = {0}, cars = 0;
  for (int pick = 0; pick < 1000; pick++) {
    int v = fleet_traffic_variant_for(pick);
    assert(v >= 0 && v < FLEET_VARIANTS);
    seen[v]++;
    if (v < FLEET_CAR_VARIANTS) cars++;
  }
  for (int i = 0; i < FLEET_VARIANTS; i++) assert(seen[i] > 0);
  assert(cars > 600 && cars < 800); /* about seven in ten are cars */
  /* Styles and sizes: the bus is the longest, the police car and the
   * ambulance flash, an ordinary car does not. */
  float longest = 0;
  int longest_variant = 0;
  for (int i = 0; i < FLEET_VARIANTS; i++) {
    const car_visual_t* v = &fleet.visuals[i];
    assert(v->chassis.id != 0 || v->cabin.id != 0 || v->accent.id != 0);
    assert(v->ride_height > 0.3f);
    if (v->half_extents.z > longest)
      longest = v->half_extents.z, longest_variant = i;
  }
  assert(fleet.visuals[longest_variant].style == BODY_BUS);
  assert(longest > 5.0f);
  assert(fleet.visuals[0].style == BODY_CAR && !fleet.visuals[0].emergency);
  int emergency = 0;
  for (int i = 0; i < FLEET_VARIANTS; i++)
    emergency += fleet.visuals[i].emergency;
  assert(emergency == 3); /* police, ambulance, fire engine */
  /* Lorries on tandem axles show extra wheels. */
  int tandems = 0;
  for (int i = 0; i < FLEET_VARIANTS; i++)
    tandems += fleet.visuals[i].extra_wheel_count > 0;
  assert(tandems >= 3);
  /* Emergency vehicles keep their own livery whatever paint is picked. */
  assert(fleet.visuals[FLEET_POLICE_VARIANT].own_paint &&
         !fleet.visuals[0].own_paint);
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
    /* A car turning round at a dead end sweeps across its road. */
    bool turning_round =
        hypotf(car.position.x, car.position.z - 100.0f) < 6.0f ||
        hypotf(car.position.x + 100.0f, car.position.z) < 6.0f ||
        hypotf(car.position.x - 100.0f, car.position.z) < 6.0f;
    assert(best < 2.2 && (best > 0.8 || turning_round));
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

static double distance_to_line(const float* p, double x, double z) {
  double dx = p[2] - p[0], dz = p[3] - p[1], l2 = dx * dx + dz * dz;
  double u = ((x - p[0]) * dx + (z - p[1]) * dz) / l2;
  u = u < 0 ? 0 : (u > 1 ? 1 : u);
  return hypot(x - (p[0] + u * dx), z - (p[1] + u * dz));
}

static void people_walk_on_the_pavements_at_a_walking_pace(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  pedestrians_t* people = pedestrians_create(routes, 3, world, 10, 9);
  assert(people && pedestrians_count(people) == 10);
  for (int step = 0; step < 120 * 40; step++) {
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
    if (step % 120) continue;
    for (size_t i = 0; i < pedestrians_count(people); i++) {
      pedestrian_t p = pedestrians_get(people, i);
      /* Road 3 m each side of the centre line, pavement 2 m beyond: people
       * walk the pavement, which is 3.2 to 5 m out (a little looser at the
       * junction where the roads meet). */
      double d = 1e9;
      for (int r = 0; r < 3; r++)
        d = fmin(
            d, distance_to_line(routes[r].points, p.position.x, p.position.z));
      assert(d > 2.5 && d < 5.2);
      assert(p.speed >= 0.0f && p.speed < 3.3f);
      assert(p.position.y > 1.9f &&
             p.position.y < 2.1f); /* on the ground (at 2) */
    }
  }
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void people_go_round_the_junctions_and_turn_back_at_dead_ends(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  pedestrians_t* people = pedestrians_create(routes, 3, world, 6, 21);
  int visited[3] = {0, 0, 0};
  float furthest_west = 0;
  for (int step = 0; step < 120 * 600; step++) {
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
    if (step % 60) continue;
    for (size_t i = 0; i < pedestrians_count(people); i++) {
      pedestrian_t p = pedestrians_get(people, i);
      visited[p.route]++;
      furthest_west = fminf(furthest_west, p.position.x);
    }
  }
  assert(visited[0] > 0 && visited[1] > 0 && visited[2] > 0);
  assert(furthest_west > -103.0f); /* never wandered off the end of a road */
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void
a_person_stops_when_the_car_is_close_and_the_legs_swing_as_they_walk(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 1, 4);
  pedestrians_place(people, 0, 0, 1, 1, 30.0f);
  float phase_before = pedestrians_get(people, 0).phase;
  for (int step = 0; step < 120 * 5; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  pedestrian_t walking = pedestrians_get(people, 0);
  assert(walking.speed > 0.8f && walking.phase > phase_before + 4.0f);
  /* A car pulls up two metres away. */
  vec3_t near_by = vec3(walking.position.x + 1.5f, 2.0f, walking.position.z);
  for (int step = 0; step < 120 * 3; step++)
    pedestrians_step(people, near_by, 1.0f / 120.0f);
  pedestrian_t waiting = pedestrians_get(people, 0);
  assert(waiting.speed < 0.05f);
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void a_car_at_speed_knocks_a_person_down_and_they_lie_still(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 1, 4);
  pedestrians_place(people, 0, 0, 1, 1, 30.0f);
  pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  vec3_t at = pedestrians_get(people, 0).position;
  vec3_t half = vec3(0.9f, 0.7f, 2.2f);
  /* A crawl does nothing, and a car beside them misses. */
  assert(pedestrians_hit(people, at, 0, half, vec3(0, 0, -1.0f)) == 0);
  vec3_t beside = vec3(at.x + 4.0f, at.y, at.z);
  assert(pedestrians_hit(people, beside, 0, half, vec3(0, 0, -12.0f)) == 0);
  /* At speed through them: down, thrown on, and flat soon after. */
  assert(pedestrians_hit(people, at, 0, half, vec3(0, 0, -12.0f)) == 1);
  assert(pedestrians_hit(people, at, 0, half, vec3(0, 0, -12.0f)) == 0);
  for (int step = 0; step < 120 * 2; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  pedestrian_t down = pedestrians_get(people, 0);
  assert(down.fall > 0.99f && down.speed == 0);
  assert(down.position.z < at.z - 1.0f);
  vec3_t resting = down.position;
  for (int step = 0; step < 120 * 4; step++)
    pedestrians_step(people, vec3(0, 0, 0), 1.0f / 120.0f);
  pedestrian_t later = pedestrians_get(people, 0);
  assert(fabsf(later.position.z - resting.z) < 0.5f && later.fall > 0.99f);
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void a_person_who_is_hit_thrashes_their_limbs_then_lies_still(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 1, 4);
  pedestrians_place(people, 0, 0, 1, 1, 30.0f);
  pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).flail == 0);
  vec3_t at = pedestrians_get(people, 0).position;
  assert(pedestrians_hit(people, at, 0, vec3(0.9f, 0.7f, 2.2f),
                         vec3(0, 0, -14.0f)) == 1);
  /* Thrashing in the air, the limbs moving all the while. */
  float phase = pedestrians_get(people, 0).phase;
  for (int step = 0; step < 30; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).flail == 1.0f &&
         pedestrians_get(people, 0).phase > phase + 1.0f);
  /* Still after a few seconds. */
  for (int step = 0; step < 120 * 5; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).flail == 0);
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void a_person_hit_fast_is_thrown_up_and_somersaults_before_landing(
    void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 1, 4);
  pedestrians_place(people, 0, 0, 1, 1, 30.0f);
  pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  vec3_t at = pedestrians_get(people, 0).position;
  assert(pedestrians_hit(people, at, 0, vec3(0.9f, 0.7f, 2.2f),
                         vec3(0, 0, -14.0f)) == 1);
  float highest = 0, most_turned = 0;
  for (int step = 0; step < 120 * 3; step++) {
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
    pedestrian_t p = pedestrians_get(people, 0);
    highest = fmaxf(highest, p.lift);
    most_turned = fmaxf(most_turned, p.fall);
  }
  assert(highest > 1.5f && most_turned > 4.0f);
  pedestrian_t landed = pedestrians_get(people, 0);
  assert(landed.lift == 0 && fabsf(landed.fall - 1.0f) < 0.001f);
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void a_bullet_hits_the_first_person_in_its_line_and_no_one_beside_it(
    void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 1, 4);
  pedestrians_place(people, 0, 0, 1, 1, 30.0f);
  pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  vec3_t at = pedestrians_get(people, 0).position;
  float distance = 0;
  /* Beside, behind and short of them: nothing. */
  assert(!pedestrians_shoot(people, vec3(at.x + 5, at.y + 1.3f, at.z - 10),
                            vec3(0, 0, 1), 40.0f, &distance));
  assert(!pedestrians_shoot(people, vec3(at.x, at.y + 1.3f, at.z + 10),
                            vec3(0, 0, 1), 40.0f, &distance));
  assert(!pedestrians_shoot(people, vec3(at.x, at.y + 1.3f, at.z - 10),
                            vec3(0, 0, -1), 40.0f, &distance));
  assert(!pedestrians_shoot(people, vec3(at.x, at.y + 1.3f, at.z - 10),
                            vec3(0, 0, 1), 5.0f, &distance));
  /* Dead on: down, at about the range. */
  assert(pedestrians_shoot(people, vec3(at.x, at.y + 1.3f, at.z - 10),
                           vec3(0, 0, 1), 40.0f, &distance));
  assert(fabsf(distance - 10.0f) < 0.6f);
  for (int step = 0; step < 120; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).fall > 0.99f);
  /* Nobody is hit twice. */
  assert(!pedestrians_shoot(people, vec3(at.x, at.y + 1.3f, at.z - 10),
                            vec3(0, 0, 1), 40.0f, &distance));
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void people_near_trouble_run_and_then_calm_down(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 1, 4);
  pedestrians_place(people, 0, 0, 1, 1, 30.0f);
  for (int step = 0; step < 120 * 3; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  pedestrian_t calm = pedestrians_get(people, 0);
  /* Far from it: no change. Near it: much quicker. */
  pedestrians_alarm(people, vec3(calm.position.x + 100, 0, calm.position.z),
                    20.0f);
  for (int step = 0; step < 120 * 2; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).speed < calm.speed * 1.3f);
  pedestrians_alarm(people, pedestrians_get(people, 0).position, 20.0f);
  for (int step = 0; step < 120 * 2; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).speed > calm.speed * 2.0f);
  /* After a few seconds they walk again. */
  for (int step = 0; step < 120 * 10; step++)
    pedestrians_step(people, vec3(1000, 0, 1000), 1.0f / 120.0f);
  assert(pedestrians_get(people, 0).speed < calm.speed * 1.3f);
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void people_come_in_sizes_and_paces_and_some_walk_in_pairs(void) {
  /* Adults, children, elders and joggers all occur; children are smaller,
   * elders slower than adults and joggers fastest. */
  int count[4] = {0, 0, 0, 0};
  float smallest_adult = 9, tallest_child = 0;
  for (int outfit = 0; outfit < 1000; outfit++) {
    person_kind_t kind = pedestrian_kind(outfit);
    count[kind]++;
    float s = pedestrian_scale(outfit);
    assert(s > 0.55f && s < 1.12f);
    if (kind == PERSON_CHILD) tallest_child = fmaxf(tallest_child, s);
    if (kind == PERSON_ADULT) smallest_adult = fminf(smallest_adult, s);
  }
  assert(count[PERSON_ADULT] > 600 && count[PERSON_CHILD] > 60 &&
         count[PERSON_ELDER] > 20 && count[PERSON_JOGGER] > 20);
  assert(tallest_child < smallest_adult);
  physics_world_t* world = ground_world();
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  pedestrians_t* people = pedestrians_create(routes, 3, world, 300, 5);
  float slowest = 9, fastest = 0;
  int pairs = 0;
  for (size_t i = 0; i < pedestrians_count(people); i++) {
    pedestrian_t a = pedestrians_get(people, i);
    float v = a.speed;
    person_kind_t kind = pedestrian_kind(a.outfit);
    if (kind == PERSON_ELDER) slowest = fminf(slowest, v);
    if (kind == PERSON_JOGGER) fastest = fmaxf(fastest, v);
    if (i % 5 == 4) {
      pedestrian_t mate = pedestrians_get(people, i - 1);
      if (mate.route == a.route &&
          vec3_distance(mate.position, a.position) < 1.0f)
        pairs++;
    }
  }
  assert(slowest < 1.15f && fastest > 2.3f);
  assert(pairs > 20); /* most of the fifths walk beside the one before */
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void nobody_starts_out_in_the_road_or_the_junction(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  for (uint32_t seed = 1; seed <= 20; seed++) {
    pedestrians_t* people = pedestrians_create(routes, 3, world, 200, seed);
    for (size_t i = 0; i < pedestrians_count(people); i++) {
      pedestrian_t p = pedestrians_get(people, i);
      double d = 1e9;
      for (int r = 0; r < 3; r++)
        d = fmin(
            d, distance_to_line(routes[r].points, p.position.x, p.position.z));
      assert(d > 2.9); /* the road is 3 m each side of its centre line */
    }
    pedestrians_destroy(people);
  }
  physics_world_destroy(world);
}

static void people_cross_at_a_zebra_crossing_and_cars_give_way(void) {
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)}; /* z = 0, 6 m wide */
  pedestrians_t* people = pedestrians_create(routes, 1, world, 60, 17);
  float crossing[2] = {-50.0f, 0.0f};
  pedestrians_set_crossings(people, crossing, 1);
  float far_player_z = 500.0f;
  int most_in_road = 0;
  bool crossed_over = false;
  float side_before[60];
  for (size_t i = 0; i < pedestrians_count(people); i++)
    side_before[i] = pedestrians_get(people, i).position.z;
  for (int step = 0; step < 120 * 90; step++) {
    pedestrians_step(people, vec3(0, 0, far_player_z), 1.0f / 120.0f);
    if (step % 30) continue;
    vec3_t walkers[64];
    int in_road = (int)pedestrians_crossing(people, walkers, 64);
    most_in_road = in_road > most_in_road ? in_road : most_in_road;
    for (int k = 0; k < in_road; k++) {
      /* Crossing means on the carriageway, at the crossing. */
      assert(fabsf(walkers[k].z) < 4.1f);
      assert(fabsf(walkers[k].x + 50.0f) < 3.0f);
    }
    for (size_t i = 0; i < pedestrians_count(people); i++) {
      float z = pedestrians_get(people, i).position.z;
      if (side_before[i] * z < 0 && fabsf(z) > 3.0f) crossed_over = true;
    }
  }
  assert(most_in_road >= 1 && crossed_over);
  /* Nobody starts across with the player's car close by. */
  pedestrians_t* wary = pedestrians_create(routes, 1, world, 60, 17);
  pedestrians_set_crossings(wary, crossing, 1);
  int started = 0;
  for (int step = 0; step < 120 * 60; step++) {
    pedestrians_step(wary, vec3(-50, 0, 8), 1.0f / 120.0f);
    vec3_t walkers[64];
    started += (int)pedestrians_crossing(wary, walkers, 64);
  }
  assert(started == 0);
  pedestrians_destroy(wary);
  pedestrians_destroy(people);
  physics_world_destroy(world);
}

static void cars_stop_for_people_in_the_road(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)}; /* one way, east */
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 3);
  vec3_t walker = vec3(-40.0f, 2.0f, 1.9f); /* in the car's lane, ahead */
  traffic_set_walkers(traffic, &walker, 1);
  traffic_car_place(traffic, 0, 0, 1, 10.0f);
  for (int step = 0; step < 120 * 15; step++)
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
  traffic_car_t waiting = traffic_car(traffic, 0);
  assert(waiting.speed < 0.2f && waiting.position.x < -42.0f);
  traffic_set_walkers(traffic, NULL, 0);
  float fastest = 0;
  for (int step = 0; step < 120 * 6; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    fastest = fmaxf(fastest, traffic_car(traffic, 0).speed);
  }
  assert(fastest > 3.0f); /* it goes on once they have crossed */
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void people_are_drawn_only_when_near(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  pedestrians_t* people = pedestrians_create(routes, 1, world, 4, 8);
  pedestrian_meshes_t meshes = pedestrian_meshes_create(renderer);
  assert(meshes.torso.id && meshes.head.id && meshes.leg.id && meshes.arm.id);
  int near_drawn =
      pedestrians_draw(people, &meshes, renderer, vec3(-50, 3, 0), 120);
  int far_drawn =
      pedestrians_draw(people, &meshes, renderer, vec3(900, 3, 900), 120);
  assert(near_drawn == 4 && far_drawn == 0);
  pedestrian_meshes_destroy(&meshes, renderer);
  pedestrians_destroy(people);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void sliding_wheels_leave_marks_that_overwrite_the_oldest(void) {
  static skid_marks_t marks;
  skid_marks_reset(&marks);
  /* Rolling: nothing. */
  for (int i = 0; i < 10; i++)
    skid_marks_wheel(&marks, 0, vec3(i * 0.5f, 0, 0), false);
  assert(marks.count == 0);
  /* Sliding: a mark per hop after the first contact. */
  for (int i = 0; i < 10; i++)
    skid_marks_wheel(&marks, 0, vec3(i * 0.5f, 0, 0), true);
  assert(marks.count == 9);
  assert(near_to(marks.marks[0].from.x, 0.0f, 1e-4f) &&
         near_to(marks.marks[0].to.x, 0.5f, 1e-4f));
  /* Tiny hops wait until the wheel has gone on, and a leap starts again. */
  skid_marks_wheel(&marks, 1, vec3(0, 0, 0), true);
  skid_marks_wheel(&marks, 1, vec3(0.01f, 0, 0), true);
  assert(marks.count == 9);
  skid_marks_wheel(&marks, 1, vec3(50, 0, 0), true);
  assert(marks.count == 9);
  /* The oldest are overwritten once full. */
  for (int i = 0; i < SKID_MARK_CAPACITY + 50; i++)
    skid_marks_wheel(&marks, 2, vec3(i * 0.3f, 0, 5), true);
  assert(marks.count == SKID_MARK_CAPACITY);
  /* Drawn when near. */
  renderer_t* renderer = renderer_create(64, 64);
  mesh_handle_t strip = skid_mark_mesh_create(renderer);
  assert(strip.id);
  int near_drawn = skid_marks_draw(&marks, strip, renderer, vec3(0, 1, 5), 40);
  int far_drawn =
      skid_marks_draw(&marks, strip, renderer, vec3(900, 1, 900), 40);
  assert(near_drawn > 5 && far_drawn == 0);
  renderer_release_mesh(renderer, strip);
  renderer_destroy(renderer);
}

static void a_signal_stops_only_the_traffic_it_faces_and_its_own_group(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)}; /* one way, east */
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 3);
  /* Time 0: group 0 is red, group 1 green. Three heads ahead of the car: one
   * for southbound traffic, one for group 1, and one for any heading. */
  city_signal_t crossing = {-40.0f, 0.0f, 0.0f, 1.0f, 0};
  city_signal_t other_group = {-40.0f, 0.0f, 1.0f, 0.0f, 1};
  traffic_set_signals(traffic, &crossing, 1);
  traffic_set_signal_time(traffic, 0.0f);
  traffic_car_place(traffic, 0, 0, 1, 10.0f);
  for (int step = 0; step < 120 * 9; step++)
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
  assert(traffic_car(traffic, 0).position.x > -41.0f); /* drove straight on */
  traffic_set_signals(traffic, &other_group, 1);
  traffic_car_place(traffic, 0, 0, 1, 10.0f);
  for (int step = 0; step < 120 * 9; step++)
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
  assert(traffic_car(traffic, 0).position.x > -41.0f);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void some_emergency_vehicles_are_on_a_call_and_can_be_heard(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 400, 11);
  size_t emergency = 0, responding = 0, others_responding = 0;
  vec3_t where = vec3_zero();
  for (size_t i = 0; i < traffic_car_count(traffic); i++) {
    bool services = fleet.visuals[traffic_car(traffic, i).variant].emergency;
    emergency += services;
    if (traffic_car_responding(traffic, i)) {
      responding++;
      where = traffic_car(traffic, i).position;
      if (!services) others_responding++;
    }
  }
  assert(emergency > 5 && others_responding == 0);
  assert(responding > 0 && responding < emergency); /* some, not all */
  assert(traffic_nearest_siren(traffic, where) < 0.5f);
  assert(traffic_nearest_siren(traffic, vec3(5000, 0, 5000)) > 1000.0f);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void a_car_signals_before_it_turns_and_stops_once_round(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  /* East along the west road to the junction, then the only way on is the
   * south road: a turn to the right (z is south). */
  city_route_t routes[2] = {route(west, 1, 2, 1), route(south, 2, 4, 1)};
  traffic_t* traffic = traffic_create(routes, 2, world, &fleet, 1, 3);
  traffic_car_place(traffic, 0, 0, 1, 40.0f);
  traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(), 1.0f / 120.0f);
  assert(traffic_car(traffic, 0).indicator == 0); /* far from the corner */
  bool signalled = false, still_signalling_after = false;
  for (int step = 0; step < 120 * 14; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    traffic_car_t car = traffic_car(traffic, 0);
    if (car.route == 0 && car.indicator == 1) signalled = true;
    if (car.route == 1 && car.position.z > 12.0f && car.indicator != 0)
      still_signalling_after = true;
  }
  assert(signalled && !still_signalling_after);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void a_car_follows_the_curve_through_a_corner_without_jumping(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[2] = {route(west, 1, 2, 1), route(south, 2, 4, 1)};
  traffic_t* traffic = traffic_create(routes, 2, world, &fleet, 1, 3);
  traffic_car_place(traffic, 0, 0, 1, 40.0f);
  traffic_car_t before = traffic_car(traffic, 0);
  float worst_turn = 0, worst_step = 0, turned = 0;
  for (int step = 0; step < 120 * 14; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    traffic_car_t car = traffic_car(traffic, 0);
    float turn = car.yaw - before.yaw;
    turn -= 6.2831853f * floorf(turn / 6.2831853f + 0.5f);
    float moved = hypotf(car.position.x - before.position.x,
                         car.position.z - before.position.z);
    if (fabsf(turn) > worst_turn) worst_turn = fabsf(turn);
    if (moved > worst_step) worst_step = moved;
    turned += turn;
    before = car;
  }
  /* A quarter turn, made gradually and with no teleporting. */
  assert(fabsf(fabsf(turned) - 1.5708f) < 0.2f);
  assert(worst_turn < 0.02f);
  assert(worst_step < 0.15f);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void a_driver_held_up_by_the_player_sounds_the_horn(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)};
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 3);
  traffic_car_place(traffic, 0, 0, 1, 40.0f);
  /* The player sits in the lane a few metres ahead of the car. */
  vec3_t player = vec3(traffic_car(traffic, 0).position.x + 9.0f, 0.5f,
                       traffic_car(traffic, 0).position.z);
  float first_honk = -1;
  bool quiet_at_first = true;
  for (int step = 0; step < 120 * 12; step++) {
    traffic_step(traffic, world, player, vec3_zero(), 1.0f / 120.0f);
    bool honking = traffic_nearest_honk(traffic, player) < 100.0f;
    if (step < 120 * 2 && honking) quiet_at_first = false;
    if (honking && first_honk < 0) first_honk = (float)step / 120.0f;
  }
  assert(quiet_at_first); /* patient at first */
  assert(first_honk > 3.0f && first_honk < 8.0f);
  assert(traffic_car(traffic, 0).speed < 0.3f); /* it did not drive through */
  /* With nobody in the way there is no horn. */
  traffic_t* free_road = traffic_create(routes, 1, world, &fleet, 1, 3);
  traffic_car_place(free_road, 0, 0, 1, 40.0f);
  for (int step = 0; step < 120 * 8; step++) {
    traffic_step(free_road, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    assert(traffic_nearest_honk(free_road, vec3(0, 0, 0)) > 1e6f);
  }
  traffic_destroy(free_road, world);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void the_satnav_finds_the_way_round_by_road_respecting_one_ways(void) {
  /* A square ring of one-way roads, anticlockwise on the map: east along the
   * top, south down the right, west along the bottom, north up the left. */
  static float top[4] = {0, 0, 100, 0}, right[4] = {100, 0, 100, 100},
               bottom[4] = {100, 100, 0, 100}, left_side[4] = {0, 100, 0, 0};
  city_route_t ring[4] = {route(top, 1, 2, 1), route(right, 2, 3, 1),
                          route(bottom, 3, 4, 1), route(left_side, 4, 1, 1)};
  nav_t* nav = nav_create(ring, 4);
  assert(nav);
  float path[64 * 2], length = 0;
  /* Ahead: along the top, then down the right. */
  size_t n = nav_path(nav, 10, 1, 100, 60, path, 64, &length);
  assert(n >= 3 && fabsf(length - 150.0f) < 3.0f);
  assert(fabsf(path[0] - 10) < 2 && fabsf(path[2 * n - 2] - 100) < 2 &&
         fabsf(path[2 * n - 1] - 60) < 2);
  /* Behind on the same one-way road: all the way round. */
  n = nav_path(nav, 50, 0, 10, 0, path, 64, &length);
  assert(n >= 5 && fabsf(length - 360.0f) < 4.0f);
  bool via_far_corner = false;
  for (size_t i = 0; i < n; i++)
    if (fabsf(path[2 * i] - 100) < 1 && fabsf(path[2 * i + 1] - 100) < 1)
      via_far_corner = true;
  assert(via_far_corner);
  /* Two-way roads can be driven either way. */
  static float two_way[4] = {0, 0, 100, 0};
  city_route_t street[1] = {route(two_way, 5, 6, 0)};
  nav_t* simple = nav_create(street, 1);
  n = nav_path(simple, 80, 0, 20, 0, path, 64, &length);
  assert(n >= 2 && fabsf(length - 60.0f) < 2.0f);
  nav_destroy(simple);
  /* Roads that do not join have no way between them. */
  static float island[4] = {500, 500, 600, 500};
  city_route_t two[2] = {route(top, 1, 2, 0), route(island, 7, 8, 0)};
  nav_t* split = nav_create(two, 2);
  assert(nav_path(split, 10, 0, 550, 500, path, 64, &length) == 0);
  nav_destroy(split);
  nav_destroy(nav);
  /* No drivable road at all, no graph. */
  city_route_t path_only = route(top, 1, 2, 0);
  path_only.drivable = 0;
  assert(nav_create(&path_only, 1) == NULL);
}

static void police_after_the_player_take_the_way_that_leads_to_them(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  nav_t* nav = nav_create(routes, 3);
  traffic_t* traffic = traffic_create(routes, 3, world, &fleet, 1, 3);
  traffic_set_nav(traffic, nav);
  traffic_car_set_kind(traffic, 0, FLEET_POLICE_VARIANT, false);
  traffic_car_place(traffic, 0, 0, 1, 30.0f); /* east, towards the junction */
  vec3_t player = vec3(0.0f, 0.5f, 85.0f);    /* far down the south road */
  assert(traffic_pursuer_count(traffic) == 0);
  assert(traffic_nearest_pursuer(traffic, player) > 1e8f);
  traffic_set_pursuit(traffic, 1);
  float fastest = 0;
  for (int step = 0; step < 120 * 25; step++) {
    traffic_step(traffic, world, player, vec3_zero(), 1.0f / 120.0f);
    fastest = fmaxf(fastest, traffic_car(traffic, 0).speed);
  }
  traffic_car_t car = traffic_car(traffic, 0);
  assert(traffic_pursuer_count(traffic) == 1);
  assert(car.route == 2); /* it took the south road */
  assert(hypotf(car.position.x - player.x, car.position.z - player.z) < 12.0f);
  assert(car.speed < 1.0f); /* and stopped behind the player */
  assert(fastest > 12.0f);  /* quicker than the 7.5 m/s of ordinary traffic */
  assert(traffic_car_responding(traffic, 0)); /* siren sounding */
  assert(traffic_nearest_pursuer(traffic, player) < 12.0f);
  /* Called off, it goes back to ordinary driving. */
  traffic_set_pursuit(traffic, 0);
  traffic_step(traffic, world, player, vec3_zero(), 1.0f / 120.0f);
  assert(traffic_pursuer_count(traffic) == 0);
  assert(!traffic_car_responding(traffic, 0));
  traffic_destroy(traffic, world);
  nav_destroy(nav);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void a_sizeable_traffic_has_police_cars_in_it(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[3] = {route(west, 1, 2, 0), route(east, 2, 3, 0),
                            route(south, 2, 4, 0)};
  traffic_t* traffic = traffic_create(routes, 3, world, &fleet, 20, 9);
  int police = 0;
  for (size_t i = 0; i < traffic_car_count(traffic); i++)
    police += traffic_car(traffic, i).variant == FLEET_POLICE_VARIANT;
  assert(police >= 3);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void
the_law_comes_after_crashes_and_the_chase_ends_in_a_bust_or_a_getaway(void) {
  wanted_t law;
  wanted_init(&law);
  assert(law.stars == 0 && wanted_pursuers(&law) == 0);
  /* A first crime starts the chase; a second right on its heels does not
   * count, a later one adds a star. */
  assert(wanted_crime(&law) == WANTED_STARTED && law.stars == 1);
  assert(wanted_pursuers(&law) == 1);
  wanted_step(&law, 500.0f, 20.0f, 0.5f);
  wanted_crime(&law);
  assert(law.stars == 1);
  wanted_step(&law, 500.0f, 20.0f, 1.5f);
  wanted_crime(&law);
  assert(law.stars == 2 && wanted_pursuers(&law) == 1);
  for (int i = 0; i < 5; i++) {
    wanted_step(&law, 500.0f, 20.0f, 2.0f);
    wanted_crime(&law);
  }
  assert(law.stars == WANTED_MAX_STARS && wanted_pursuers(&law) == 3);
  /* Caught standing: busted after two seconds, and the stars clear. */
  wanted_event_t event = WANTED_NOTHING;
  for (int i = 0; i < 10 && event == WANTED_NOTHING; i++)
    event = wanted_step(&law, 6.0f, 1.0f, 0.25f);
  assert(event == WANTED_BUSTED && law.stars == 0);
  /* A police car alongside a player who keeps moving does not catch them. */
  wanted_crime(&law);
  for (int i = 0; i < 20; i++)
    assert(wanted_step(&law, 6.0f, 15.0f, 0.5f) == WANTED_NOTHING);
  assert(law.stars == 1);
  wanted_init(&law);
  wanted_crime(&law);
  /* Police a long way off that have not found the player yet are no
   * getaway: the chase waits for them (but not for ever). */
  event = WANTED_NOTHING;
  float waited = 0;
  for (; waited < 120.0f && event == WANTED_NOTHING; waited += 0.5f)
    event = wanted_step(&law, 400.0f, 30.0f, 0.5f);
  assert(event == WANTED_ESCAPED && waited > 60.0f);
  wanted_crime(&law);
  law.since_crime = 9.0f;
  /* Once they have come close, far away for twelve seconds is a getaway. */
  wanted_step(&law, 100.0f, 30.0f, 0.5f);
  event = WANTED_NOTHING;
  float elapsed = 0;
  for (; elapsed < 30.0f && event == WANTED_NOTHING; elapsed += 0.5f)
    event = wanted_step(&law, 400.0f, 30.0f, 0.5f);
  assert(event == WANTED_ESCAPED && elapsed > 11.0f && elapsed < 14.0f);
  assert(law.stars == 0);
  /* Nothing happens while the law is not interested. */
  assert(wanted_step(&law, 1.0f, 0.0f, 5.0f) == WANTED_NOTHING);
}

static void the_player_on_foot_walks_runs_turns_and_is_stopped_by_walls(void) {
  physics_world_t* world = ground_world(); /* ground at y = 2 */
  /* A wall across the way, 5 m ahead (north, -z), and a kerb 0.15 m high. */
  physics_add_static_box(world, transform_at(vec3(0, 3, -6), quat_identity()),
                         vec3(10, 1.5f, 0.3f), COLLISION_LAYER_BUILDING, 0.6f,
                         0);
  walker_t walker;
  walker_init(&walker, vec3(0, 2, 0), 0.0f);
  walker_input_t stand = {0, 0, false}, walk = {1, 0, false},
                 run = {1, 0, true};
  for (int i = 0; i < 120; i++) walker_step(&walker, world, stand, 1.0f / 120);
  assert(fabsf(walker.position.z) < 0.01f &&
         fabsf(walker.position.y - 2) < 0.01f);
  /* A second's walk covers about two metres, a run's second more. */
  for (int i = 0; i < 120; i++) walker_step(&walker, world, walk, 1.0f / 120);
  float walked = -walker.position.z;
  assert(walked > 1.5f && walked < 2.2f);
  float before = walker.position.z;
  for (int i = 0; i < 120; i++) walker_step(&walker, world, run, 1.0f / 120);
  assert(before - walker.position.z > 3.5f || walker.position.z < -4.0f);
  /* The wall stops it, with room to spare, and it does not go through. */
  for (int i = 0; i < 600; i++) walker_step(&walker, world, run, 1.0f / 120);
  assert(walker.position.z > -6.3f + 0.0f - 0.01f && walker.position.z < -5.2f);
  assert(fabsf(walker.position.y - 2) < 0.05f);
  /* Turning to the right turns the way it faces to the right. */
  walker_init(&walker, vec3(0, 2, 50), 0.0f);
  walker_input_t right = {0, 1, false};
  for (int i = 0; i < 120; i++) walker_step(&walker, world, right, 1.0f / 120);
  vec3_t facing = walker_forward(&walker);
  assert(facing.x > 0.2f); /* north turned towards the east */
  /* Backing away is slower than going forward. */
  walker_init(&walker, vec3(0, 2, 50), 0.0f);
  walker_input_t back = {-1, 0, false};
  for (int i = 0; i < 240; i++) walker_step(&walker, world, back, 1.0f / 120);
  assert(walker.position.z - 50 > 1.0f && walker.position.z - 50 < 2.5f);
  /* Out of a parked car's way. */
  walker_init(&walker, vec3(0.2f, 2, 60), 0.0f);
  walker_push_out_of_box(&walker, vec3(0, 2.7f, 60), 0.0f,
                         vec3(0.9f, 0.7f, 2.2f));
  assert(fabsf(walker.position.x) > 1.2f ||
         fabsf(walker.position.z - 60) > 2.5f);
  physics_world_destroy(world);
}

static void a_standing_car_is_gone_round_when_the_other_lane_is_clear(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  for (int oncoming = 0; oncoming < 2; oncoming++) {
    /* A two-way road, 6 m wide: a van standing in the lane, a car behind it,
     * and, the second time, another standing in the other lane ahead. */
    static float long_road[4] = {-300, 0, 0, 0};
    city_route_t routes[1] = {route(long_road, 1, 2, 0)};
    traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 3, 11);
    traffic_car_set_kind(traffic, 0, 0, false); /* hatchback behind */
    traffic_car_set_kind(traffic, 1, 2, false); /* van standing */
    traffic_car_set_kind(traffic, 2, 0, false); /* oncoming or out of the way */
    traffic_car_place(traffic, 0, 0, 1, 40.0f);
    traffic_car_place(traffic, 1, 0, 1, 70.0f);
    traffic_car_place(traffic, 2, 0, -1, oncoming ? 200.0f : 290.0f);
    traffic_car_hold(traffic, 1, true);
    if (oncoming) traffic_car_hold(traffic, 2, true);
    float widest = 0;
    bool passed = false, passed_before_clear = false;
    for (int step = 0; step < 120 * 25; step++) {
      traffic_step(traffic, world, vec3(0, 0, 2000), vec3_zero(),
                   1.0f / 120.0f);
      traffic_car_t me = traffic_car(traffic, 0), van = traffic_car(traffic, 1);
      if (me.position.x > van.position.x - 2.0f &&
          me.position.x < van.position.x + 2.0f)
        widest = fmaxf(widest, fabsf(me.position.z - van.position.z));
      if (me.position.x > van.position.x + 4.0f) passed = true;
      if (oncoming && step < 120 * 12 && passed) passed_before_clear = true;
      assert(hypotf(me.position.x - van.position.x,
                    me.position.z - van.position.z) > 1.5f); /* never inside */
    }
    if (!oncoming) {
      assert(passed);        /* it went round the van */
      assert(widest > 2.4f); /* out in the other lane to do it */
    } else {
      assert(!passed &&
             !passed_before_clear); /* the other lane was not clear */
    }
    traffic_destroy(traffic, world);
  }
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void cars_pull_over_for_an_emergency_vehicle_that_passes_them(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  /* A two-way road, 6 m wide, heading east: an ordinary car with an
   * ambulance on a call coming up behind it. */
  city_route_t routes[1] = {route(west, 1, 2, 0)};
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 2, 5);
  traffic_car_set_kind(traffic, 0, 0, false); /* hatchback */
  traffic_car_set_kind(traffic, 1, FLEET_AMBULANCE_VARIANT, true);
  traffic_car_place(traffic, 0, 0, 1, 45.0f);
  traffic_car_place(traffic, 1, 0, 1, 12.0f);
  float widest = 0;
  bool passed = false;
  for (int step = 0; step < 120 * 14; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    traffic_car_t car = traffic_car(traffic, 0),
                  ambulance = traffic_car(traffic, 1);
    widest = fmaxf(widest, fabsf(car.position.z - ambulance.position.z));
    if (ambulance.position.x > car.position.x + 4.0f && car.position.x > -54.0f)
      passed = true;
    /* Never inside one another. */
    assert(hypotf(car.position.x - ambulance.position.x,
                  car.position.z - ambulance.position.z) > 1.5f);
  }
  assert(passed);        /* the ambulance got by */
  assert(widest > 2.4f); /* by using the room: one to the kerb, one out */
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void a_bus_halts_at_a_bus_stop_for_a_while_and_goes_on(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)}; /* east along z = 0 */
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 5);
  traffic_car_set_kind(traffic, 0, FLEET_BUS_VARIANT, false);
  float stop[2] = {-60.0f, 3.2f}; /* on the right-hand kerb */
  traffic_set_bus_stops(traffic, stop, 1);
  traffic_car_place(traffic, 0, 0, 1, 15.0f);
  float stood = 0, last_x = -1e9f;
  bool went_on = false;
  for (int step = 0; step < 120 * 24; step++) { /* before the second lap */
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    traffic_car_t bus = traffic_car(traffic, 0);
    if (bus.speed < 0.05f && fabsf(bus.position.x - stop[0]) < 3.5f)
      stood += 1.0f / 120.0f;
    if (bus.position.x > stop[0] + 12.0f && bus.speed > 3.0f) went_on = true;
    last_x = bus.position.x;
  }
  (void)last_x;
  assert(stood > 6.0f && stood < 14.0f); /* about eight seconds */
  assert(went_on);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void cars_and_people_left_far_behind_turn_up_near_the_player(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  /* A road from 300 to 700 m and one 5 000 m from the focus at the origin. */
  static float near_road[8] = {300, 0, 400, 0, 600, 0, 700, 0};
  static float far_road[4] = {5000, 0, 5100, 0};
  city_route_t routes[2] = {route(near_road, 1, 2, 0),
                            route(far_road, 3, 4, 0)};
  routes[0].point_count = 4;
  traffic_t* traffic = traffic_create(routes, 2, world, &fleet, 5, 5);
  for (size_t i = 0; i < traffic_car_count(traffic); i++)
    traffic_car_place(traffic, i, 1, 1, 20.0f); /* all on the far road */
  traffic_set_focus(traffic, vec3(0, 2, 0));
  for (int step = 0; step < 10; step++)
    traffic_step(traffic, world, vec3(0, 2, 0), vec3_zero(), 1.0f / 120.0f);
  for (size_t i = 0; i < traffic_car_count(traffic); i++) {
    traffic_car_t car = traffic_car(traffic, i);
    assert(car.route == 0); /* on the road near the player */
    /* Beyond where cars are drawn, so none appears in sight. */
    assert(hypotf(car.position.x, car.position.z) > TRAFFIC_VISIBLE_RANGE &&
           hypotf(car.position.x, car.position.z) < 900.0f);
  }
  /* People are drawn nearer than cars, so they turn up nearer: a street from
   * 300 to 400 m is within their band. */
  static float people_street[4] = {300, 0, 400, 0};
  city_route_t streets[2] = {route(people_street, 1, 2, 0),
                             route(far_road, 3, 4, 0)};
  pedestrians_t* people = pedestrians_create(streets, 2, world, 12, 7);
  pedestrians_set_focus(people, vec3(0, 2, 0));
  for (int step = 0; step < 10; step++)
    pedestrians_step(people, vec3(0, 2, 0), 1.0f / 120.0f);
  for (size_t i = 0; i < pedestrians_count(people); i++) {
    pedestrian_t person = pedestrians_get(people, i);
    assert(hypotf(person.position.x, person.position.z) < 460.0f);
  }
  pedestrians_destroy(people);
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

static void cars_wait_at_a_red_signal_and_go_on_green(void) {
  renderer_t* renderer = renderer_create(64, 64);
  physics_world_t* world = ground_world();
  fleet_t fleet;
  assert(fleet_create(&fleet, renderer));
  city_route_t routes[1] = {route(west, 1, 2, 1)}; /* one way, east */
  traffic_t* traffic = traffic_create(routes, 1, world, &fleet, 1, 3);
  /* A head for eastbound traffic, in group 0: red at time 0. */
  city_signal_t signal = {-40.0f, 0.0f, 1.0f, 0.0f, 0};
  traffic_set_signals(traffic, &signal, 1);
  traffic_car_place(traffic, 0, 0, 1, 10.0f);
  traffic_set_signal_time(traffic, 0.0f);
  for (int step = 0; step < 120 * 25; step++)
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
  traffic_car_t waiting = traffic_car(traffic, 0);
  assert(waiting.speed < 0.2f);
  assert(waiting.position.x > -52.0f && waiting.position.x < -44.0f);
  traffic_set_signal_time(traffic, 30.0f);
  float furthest = -1e9f;
  for (int step = 0; step < 120 * 12; step++) {
    traffic_step(traffic, world, vec3(1000, 0, 1000), vec3_zero(),
                 1.0f / 120.0f);
    furthest = fmaxf(furthest, traffic_car(traffic, 0).position.x);
  }
  assert(furthest > -20.0f);
  /* The cycle: red, then red and amber, green, amber, in forty-eight seconds.
   */
  assert(!traffic_signals_open(5.0f) && !traffic_signals_open(22.0f));
  assert(traffic_signals_open(30.0f) && !traffic_signals_open(46.0f));
  assert(!traffic_signals_open(48.0f + 5.0f) &&
         traffic_signals_open(48.0f + 30.0f));
  /* The crossing street's group is green while this one is red. */
  assert(traffic_signal_open(5.0f, 1) && !traffic_signal_open(30.0f, 1));
  for (float t = 0; t < 96.0f; t += 0.5f)
    assert(!(traffic_signal_open(t, 0) && traffic_signal_open(t, 1)));
  traffic_destroy(traffic, world);
  fleet_destroy(&fleet, renderer);
  physics_world_destroy(world);
  renderer_destroy(renderer);
}

void run_traffic_tests(void) {
  RUN(cars_wait_at_a_red_signal_and_go_on_green);
  RUN(cars_and_people_left_far_behind_turn_up_near_the_player);
  RUN(a_bus_halts_at_a_bus_stop_for_a_while_and_goes_on);
  RUN(cars_pull_over_for_an_emergency_vehicle_that_passes_them);
  RUN(a_car_signals_before_it_turns_and_stops_once_round);
  RUN(a_driver_held_up_by_the_player_sounds_the_horn);
  RUN(a_standing_car_is_gone_round_when_the_other_lane_is_clear);
  RUN(the_satnav_finds_the_way_round_by_road_respecting_one_ways);
  RUN(the_player_on_foot_walks_runs_turns_and_is_stopped_by_walls);
  RUN(police_after_the_player_take_the_way_that_leads_to_them);
  RUN(a_sizeable_traffic_has_police_cars_in_it);
  RUN(the_law_comes_after_crashes_and_the_chase_ends_in_a_bust_or_a_getaway);
  RUN(a_car_follows_the_curve_through_a_corner_without_jumping);
  RUN(some_emergency_vehicles_are_on_a_call_and_can_be_heard);
  RUN(a_signal_stops_only_the_traffic_it_faces_and_its_own_group);
  RUN(sliding_wheels_leave_marks_that_overwrite_the_oldest);
  RUN(people_walk_on_the_pavements_at_a_walking_pace);
  RUN(people_go_round_the_junctions_and_turn_back_at_dead_ends);
  RUN(a_person_stops_when_the_car_is_close_and_the_legs_swing_as_they_walk);
  RUN(people_come_in_sizes_and_paces_and_some_walk_in_pairs);
  RUN(nobody_starts_out_in_the_road_or_the_junction);
  RUN(people_cross_at_a_zebra_crossing_and_cars_give_way);
  RUN(cars_stop_for_people_in_the_road);
  RUN(people_are_drawn_only_when_near);
  RUN(the_fleet_has_four_bodies_and_a_palette);
  RUN(the_road_carries_buses_lorries_and_the_emergency_services);
  RUN(parked_cars_stand_on_the_ground_and_are_solid);
  RUN(cars_follow_the_roads_on_the_right_at_a_sensible_speed);
  RUN(cars_turn_at_junctions_onto_every_road_and_keep_apart);
  RUN(a_car_at_speed_knocks_a_person_down_and_they_lie_still);
  RUN(a_person_hit_fast_is_thrown_up_and_somersaults_before_landing);
  RUN(a_person_who_is_hit_thrashes_their_limbs_then_lies_still);
  RUN(a_bullet_hits_the_first_person_in_its_line_and_no_one_beside_it);
  RUN(people_near_trouble_run_and_then_calm_down);
  RUN(a_car_stops_short_of_the_player_in_its_way);
  RUN(at_a_dead_end_a_car_turns_round_and_comes_back);
  RUN(each_car_carries_a_solid_box_that_moves_with_it);
}
