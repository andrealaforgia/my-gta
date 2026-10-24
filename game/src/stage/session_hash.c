#include <string.h>

#include "session_log.h"

static uint64_t hash_bytes(uint64_t hash, const void* bytes, size_t count) {
  const unsigned char* data = bytes;
  for (size_t i = 0; i < count; i++) {
    hash ^= data[i];
    hash *= 1099511628211ull;
  }
  return hash;
}

static uint64_t hash_vector(uint64_t hash, vec3_t value) {
  const float values[] = {value.x, value.y, value.z};
  return hash_bytes(hash, values, sizeof(values));
}

static uint64_t hash_rotation(uint64_t hash, quat_t value) {
  const float values[] = {value.x, value.y, value.z, value.w};
  return hash_bytes(hash, values, sizeof(values));
}

uint64_t session_vehicle_hash(const vehicle_params_t* p) {
  uint64_t hash = 14695981039346656037ull;
  const float values[] = {p->mass,
                          p->chassis_half_extents.x,
                          p->chassis_half_extents.y,
                          p->chassis_half_extents.z,
                          p->track_width,
                          p->wheelbase,
                          p->wheel_attach_height,
                          p->wheel_radius,
                          p->wheel_width,
                          p->suspension_rest_length,
                          p->spring_stiffness,
                          p->damper_compression,
                          p->damper_rebound,
                          p->tyre_friction,
                          p->tyre_lateral_stiffness,
                          p->rolling_resistance,
                          p->roll_centre_lift,
                          p->drag_coefficient,
                          p->max_steer_angle,
                          p->steering_rate,
                          p->steering_half_lock_speed,
                          p->brake_force,
                          p->handbrake_force,
                          p->handbrake_grip_factor,
                          p->engine_idle_rpm,
                          p->engine_max_rpm,
                          p->engine_braking,
                          p->reverse_ratio,
                          p->final_drive,
                          p->drivetrain_efficiency,
                          p->shift_up_rpm,
                          p->shift_down_rpm,
                          p->shift_time};
  const int integers[] = {p->torque_point_count, p->gear_count, (int)p->drive};
  hash = hash_bytes(hash, p->name, strlen(p->name));
  hash = hash_bytes(hash, p->body, strlen(p->body));
  hash = hash_bytes(hash, values, sizeof(values));
  hash = hash_bytes(hash, integers, sizeof(integers));
  hash = hash_bytes(hash, p->gear_ratios, sizeof(p->gear_ratios));
  for (int i = 0; i < VEHICLE_MAX_TORQUE_POINTS; i++) {
    const float point[] = {p->torque_curve[i].rpm, p->torque_curve[i].torque};
    hash = hash_bytes(hash, point, sizeof(point));
  }
  return hash;
}

static uint64_t hash_body(uint64_t hash, const rigid_body_t* body) {
  hash = hash_vector(hash, body->position);
  hash = hash_rotation(hash, body->rotation);
  hash = hash_vector(hash, body->velocity);
  hash = hash_vector(hash, body->angular_velocity);
  hash = hash_vector(hash, body->force);
  hash = hash_vector(hash, body->torque);
  hash = hash_vector(hash, body->previous.position);
  return hash_rotation(hash, body->previous.rotation);
}

static uint64_t hash_traffic(uint64_t hash, const driving_stage_t* stage) {
  size_t count = stage->traffic ? traffic_car_count(stage->traffic) : 0;
  hash = hash_bytes(hash, &count, sizeof(count));
  for (size_t i = 0; i < count; i++) {
    traffic_car_t car = traffic_car(stage->traffic, i);
    hash = hash_vector(hash, car.position);
    const float values[] = {car.yaw, car.speed, car.wheel_turn};
    const int flags[] = {car.route,   car.variant,   car.paint,
                         car.braking, car.indicator, car.pursuing};
    hash = hash_bytes(hash, values, sizeof(values));
    hash = hash_bytes(hash, flags, sizeof(flags));
  }
  count = stage->people ? pedestrians_count(stage->people) : 0;
  hash = hash_bytes(hash, &count, sizeof(count));
  for (size_t i = 0; i < count; i++) {
    pedestrian_t person = pedestrians_get(stage->people, i);
    hash = hash_vector(hash, person.position);
    const float values[] = {person.yaw,  person.speed, person.phase,
                            person.fall, person.lift,  person.flail};
    const int flags[] = {person.outfit, person.route};
    hash = hash_bytes(hash, values, sizeof(values));
    hash = hash_bytes(hash, flags, sizeof(flags));
  }
  for (size_t i = 0; i < stage->parked.count; i++) {
    const parked_car_t* car = &stage->parked.cars[i];
    hash = hash_vector(hash, car->position);
    const int flags[] = {car->variant, car->paint, car->taken};
    hash = hash_bytes(hash, flags, sizeof(flags));
    hash = hash_bytes(hash, &car->yaw, sizeof(car->yaw));
  }
  return hash;
}

static uint64_t hash_aircraft(uint64_t hash, const driving_stage_t* stage) {
  for (size_t i = 0; i < stage->aircraft_count; i++) {
    const aircraft_t* a = &stage->aircraft[i];
    hash = hash_vector(hash, a->position);
    const float values[] = {a->yaw,         a->pitch,    a->roll,  a->speed,
                            a->climb,       a->path,     a->power, a->rotor,
                            a->rotor_speed, a->yaw_rate, a->gear};
    const int flags[] = {a->kind, a->on_ground, a->gear_down, a->doomed};
    hash = hash_bytes(hash, values, sizeof(values));
    hash = hash_bytes(hash, flags, sizeof(flags));
  }
  for (size_t i = 0; i < 8; i++) {
    int active = stage->missiles[i].active;
    hash = hash_bytes(hash, &active, sizeof(active));
    hash = hash_vector(hash, stage->missiles[i].position);
    hash = hash_vector(hash, stage->missiles[i].velocity);
    hash = hash_bytes(hash, &stage->missiles[i].age, sizeof(float));
  }
  const float chute[] = {stage->chute.time, stage->chute.height,
                         stage->chute.bank};
  hash = hash_bytes(hash, chute, sizeof(chute));
  return hash_vector(hash, stage->chute.velocity);
}

static uint64_t hash_world(uint64_t hash, const driving_stage_t* stage) {
  size_t count = physics_collider_count(stage->physics);
  hash = hash_bytes(hash, &count, sizeof(count));
  for (size_t i = 0; i < count; i++) {
    const static_collider_t* c = physics_collider_at(stage->physics, i);
    int used = c->used;
    hash = hash_bytes(hash, &used, sizeof(used));
    if (!used) continue;
    hash = hash_vector(hash, c->bounds.min);
    hash = hash_vector(hash, c->bounds.max);
  }
  for (size_t i = 0; i < stage->scene.pole_count; i++) {
    const pole_t* pole = &stage->scene.poles[i];
    int state = pole->state;
    hash = hash_bytes(hash, &state, sizeof(state));
    hash = hash_bytes(hash, &pole->fall, sizeof(pole->fall));
    hash = hash_vector(hash, pole->pushed);
  }
  if (stage->scene.package) {
    for (size_t i = 0; i < stage->scene.package->chunk_count; i++) {
      int loaded = stage->scene.chunk_states[i].loaded;
      hash = hash_bytes(hash, &loaded, sizeof(loaded));
    }
  }
  return hash;
}

uint64_t session_state_hash(const driving_stage_t* s) {
  uint64_t hash = 14695981039346656037ull;
  hash = hash_bytes(hash, &s->seed, sizeof(s->seed));
  uint64_t vehicle = session_vehicle_hash(&s->vehicle.params);
  hash = hash_bytes(hash, &vehicle, sizeof(vehicle));
  const float values[] = {s->rain,
                          s->rain_target,
                          s->rain_clock,
                          s->hour,
                          s->hour_seconds,
                          s->simulated_seconds,
                          s->signal_clock,
                          s->burning,
                          s->blast,
                          s->entering,
                          s->door_clock,
                          s->shot_cooldown,
                          s->shot_flash,
                          s->punch_clock,
                          s->punch_rest,
                          s->missile_cooldown,
                          s->walker.yaw,
                          s->walker.speed,
                          s->walker.fall,
                          s->walker.phase,
                          s->wanted.since_crime,
                          s->wanted.caught_for,
                          s->wanted.out_of_reach,
                          s->wanted.chase_time,
                          s->vehicle.rpm,
                          s->vehicle.throttle,
                          s->vehicle.brake,
                          s->vehicle.steering,
                          s->vehicle.shift_timer,
                          s->vehicle.steering_filter.value,
                          s->vehicle.forward_speed,
                          s->vehicle.skid_intensity,
                          s->vehicle.impact_impulse,
                          s->vehicle.damage};
  const int flags[] = {s->scene_kind,        s->paused,
                       s->map_open,          s->on_foot,
                       s->walker.grounded,   s->wanted.stars,
                       s->wanted.engaged,    s->vehicle.gear,
                       s->vehicle.handbrake, s->vehicle_index,
                       s->piloting,          s->air_view,
                       s->chute.active,      s->chute.opened,
                       s->wrecked,           s->blast_kills,
                       s->blast_car,         s->radio_station,
                       s->view_mode,         s->horn,
                       s->missile_next,      s->quit};
  hash = hash_bytes(hash, values, sizeof(values));
  hash = hash_bytes(hash, flags, sizeof(flags));
  hash = hash_bytes(hash, &s->clock.accumulator, sizeof(double));
  hash = hash_vector(hash, s->walker.position);
  hash = hash_vector(hash, s->entry_start);
  hash = hash_vector(hash, s->blast_at);
  int own_paint = s->car_visual.own_paint;
  hash = hash_bytes(hash, &own_paint, sizeof(own_paint));
  hash = hash_vector(hash, s->car_visual.own_colour);
  hash = hash_vector(hash, s->car_visual.paint.base_color);
  hash = hash_body(hash, physics_body(s->physics, s->vehicle.body));
  for (int i = 0; i < VEHICLE_WHEEL_COUNT; i++) {
    const wheel_state_t* w = &s->vehicle.wheels[i];
    const float wheel[] = {
        w->compression,      w->compression_rate, w->load,
        w->surface_friction, w->steer_angle,      w->spin,
        w->rotation,         w->slip_angle,       w->grip_use};
    const int flags[] = {w->grounded, w->skidding};
    hash = hash_bytes(hash, wheel, sizeof(wheel));
    hash = hash_bytes(hash, flags, sizeof(flags));
    hash = hash_vector(hash, w->contact_point);
    hash = hash_vector(hash, w->contact_normal);
  }
  return hash_world(hash_aircraft(hash_traffic(hash, s), s), s);
}

uint64_t session_content_hash(const driving_stage_t* stage) {
  uint64_t hash = 14695981039346656037ull;
  const city_package_t* package = stage->scene.package;
  if (!package) return hash_world(hash, stage);
  hash = hash_bytes(hash, &package->config_hash, sizeof(package->config_hash));
  hash = hash_vector(hash, package->spawn_position);
  hash = hash_bytes(hash, &package->spawn_yaw, sizeof(float));
  hash = hash_bytes(hash, &package->chunk_count, sizeof(size_t));
  hash = hash_bytes(hash, &package->route_count, sizeof(size_t));
  hash = hash_bytes(hash, &package->signal_count, sizeof(size_t));
  hash = hash_bytes(hash, &package->parked_count, sizeof(size_t));
  hash = hash_bytes(hash, &package->aircraft_count, sizeof(size_t));
  for (size_t i = 0; i < package->signal_count; i++) {
    const city_signal_t* signal = &package->signals[i];
    const float values[] = {signal->x, signal->z, signal->heading_x,
                            signal->heading_z};
    hash = hash_bytes(hash, values, sizeof(values));
    hash = hash_bytes(hash, &signal->group, sizeof(int));
  }
  for (size_t i = 0; i < package->parked_count; i++) {
    const city_parking_t* car = &package->parked[i];
    const float values[] = {car->x, car->z, car->yaw};
    hash = hash_bytes(hash, values, sizeof(values));
    hash = hash_bytes(hash, &car->variant, sizeof(int));
  }
  hash = hash_bytes(hash, package->crossings,
                    package->crossing_count * 2 * sizeof(float));
  hash = hash_bytes(hash, package->bus_stops,
                    package->bus_stop_count * 2 * sizeof(float));
  const city_airport_t* airport = &package->airport;
  const float airport_values[] = {
      airport->x,     airport->z,      airport->yaw,    airport->length,
      airport->width, airport->gate_x, airport->gate_y, airport->gate_z};
  int present = airport->present;
  hash = hash_bytes(hash, &present, sizeof(present));
  hash = hash_bytes(hash, airport_values, sizeof(airport_values));
  for (size_t i = 0; i < package->aircraft_count; i++) {
    const city_aircraft_t* aircraft = &package->aircraft[i];
    const float values[] = {aircraft->x, aircraft->y, aircraft->z,
                            aircraft->yaw};
    hash = hash_bytes(hash, &aircraft->kind, sizeof(int));
    hash = hash_bytes(hash, values, sizeof(values));
  }
  for (size_t i = 0; i < package->chunk_count; i++) {
    const city_chunk_t* chunk = &package->chunks[i];
    hash = hash_bytes(hash, &chunk->part_count, sizeof(size_t));
    hash = hash_vector(hash, chunk->bounds.min);
    hash = hash_vector(hash, chunk->bounds.max);
    for (size_t j = 0; j < chunk->part_count; j++) {
      const city_part_t* part = &chunk->parts[j];
      hash = hash_bytes(hash, &part->mesh.vertex_count, sizeof(size_t));
      hash = hash_bytes(hash, &part->mesh.index_count, sizeof(size_t));
      hash = hash_bytes(hash, &part->source_id, sizeof(part->source_id));
      hash = hash_bytes(hash, part->mesh.vertices,
                        part->mesh.vertex_count * sizeof(vertex_t));
      hash = hash_bytes(hash, part->mesh.indices,
                        part->mesh.index_count * sizeof(uint32_t));
      const int collision[] = {part->collider, part->layer};
      hash = hash_bytes(hash, collision, sizeof(collision));
      hash = hash_bytes(hash, &part->friction, sizeof(float));
    }
  }
  for (size_t i = 0; i < package->route_count; i++) {
    const city_route_t* r = &package->routes[i];
    const int flags[] = {r->drivable, r->oneway};
    const float values[] = {r->width, r->pavement};
    hash = hash_bytes(hash, flags, sizeof(flags));
    hash = hash_bytes(hash, values, sizeof(values));
    hash = hash_bytes(hash, &r->start_node, sizeof(r->start_node));
    hash = hash_bytes(hash, &r->end_node, sizeof(r->end_node));
    hash = hash_bytes(hash, r->points, r->point_count * 2 * sizeof(float));
  }
  return hash;
}
