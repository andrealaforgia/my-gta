/* Parked and moving cars: the fleet, the cars at the kerb, and traffic that
 * follows the roads, turns at junctions, keeps its distance and stops. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fleet.h"
#include "parked_cars.h"
#include "pedestrians.h"
#include "skid_marks.h"
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
  assert(fleet.visuals[4].own_paint && !fleet.visuals[0].own_paint);
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
  RUN(a_signal_stops_only_the_traffic_it_faces_and_its_own_group);
  RUN(sliding_wheels_leave_marks_that_overwrite_the_oldest);
  RUN(people_walk_on_the_pavements_at_a_walking_pace);
  RUN(people_go_round_the_junctions_and_turn_back_at_dead_ends);
  RUN(a_person_stops_when_the_car_is_close_and_the_legs_swing_as_they_walk);
  RUN(people_come_in_sizes_and_paces_and_some_walk_in_pairs);
  RUN(people_are_drawn_only_when_near);
  RUN(the_fleet_has_four_bodies_and_a_palette);
  RUN(the_road_carries_buses_lorries_and_the_emergency_services);
  RUN(parked_cars_stand_on_the_ground_and_are_solid);
  RUN(cars_follow_the_roads_on_the_right_at_a_sensible_speed);
  RUN(cars_turn_at_junctions_onto_every_road_and_keep_apart);
  RUN(a_car_stops_short_of_the_player_in_its_way);
  RUN(at_a_dead_end_a_car_turns_round_and_comes_back);
  RUN(each_car_carries_a_solid_box_that_moves_with_it);
}
