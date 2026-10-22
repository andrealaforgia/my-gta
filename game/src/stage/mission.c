#include "mission.h"

#include <math.h>

static uint32_t next_random(mission_t* m) {
  m->random = m->random * 1664525u + 1013904223u;
  return m->random >> 8;
}

void mission_init(mission_t* mission, uint32_t seed) {
  *mission = (mission_t){0};
  mission->random = seed * 2654435761u + 977u;
  mission->pause = 1.5f;
}

int mission_reward(float distance, float time_left) {
  return 100 + (int)(distance * 0.2f) + (int)(fmaxf(time_left, 0.0f) * 3.0f);
}

float mission_distance_to(const mission_t* mission, float px, float pz) {
  return hypotf(mission->x - px, mission->z - pz);
}

/* A spot on a drivable road between 250 and 1000 m away, or any 100 m or
 * more if the city has none so far. */
static bool pick_destination(mission_t* m, const city_route_t* routes,
                             size_t count, float px, float pz) {
  if (count == 0) return false;
  for (int attempt = 0; attempt < 400; attempt++) {
    const city_route_t* route = &routes[next_random(m) % count];
    if (!route->drivable || route->point_count < 2) continue;
    size_t k = next_random(m) % route->point_count;
    float x = route->points[2 * k], z = route->points[2 * k + 1];
    float d = hypotf(x - px, z - pz);
    float least = attempt < 300 ? 250.0f : 100.0f;
    float most = attempt < 300 ? 1000.0f : 3000.0f;
    if (d < least || d > most) continue;
    m->x = x, m->z = z, m->distance = d;
    return true;
  }
  return false;
}

mission_event_t mission_step(mission_t* m, const city_route_t* routes,
                             size_t route_count, float px, float pz, float dt) {
  if (m->result_time > 0) m->result_time -= dt;
  if (!m->active) {
    m->pause -= dt;
    if (m->pause > 0) return MISSION_NOTHING;
    if (!pick_destination(m, routes, route_count, px, pz)) {
      m->pause = 5.0f;
      return MISSION_NOTHING;
    }
    m->active = true;
    m->time_limit = 20.0f + m->distance / 6.0f;
    m->time_left = m->time_limit;
    return MISSION_STARTED;
  }
  m->time_left -= dt;
  if (mission_distance_to(m, px, pz) < MISSION_ARRIVAL_RADIUS) {
    m->active = false;
    m->last_reward = mission_reward(m->distance, m->time_left);
    m->cash += m->last_reward;
    m->delivered++;
    m->pause = 3.0f;
    m->result_time = 3.5f;
    return MISSION_DELIVERED;
  }
  if (m->time_left <= 0) {
    m->active = false;
    m->last_reward = 0;
    m->failed++;
    m->pause = 3.0f;
    m->result_time = 3.5f;
    return MISSION_TIMED_OUT;
  }
  return MISSION_NOTHING;
}
