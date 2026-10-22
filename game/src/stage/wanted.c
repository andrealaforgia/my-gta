#include "wanted.h"

void wanted_init(wanted_t* wanted) {
  wanted->stars = 0;
  wanted->since_crime = 99.0f;
  wanted->caught_for = 0;
  wanted->out_of_reach = 0;
  wanted->chase_time = 0;
  wanted->engaged = 0;
}

wanted_event_t wanted_crime(wanted_t* wanted) {
  if (wanted->since_crime < 1.5f) return WANTED_NOTHING;
  wanted->since_crime = 0;
  wanted->out_of_reach = 0;
  if (wanted->stars >= WANTED_MAX_STARS) return WANTED_NOTHING;
  if (wanted->stars == 0) {
    wanted->chase_time = 0;
    wanted->engaged = 0;
    wanted->caught_for = 0;
  }
  wanted->stars++;
  return wanted->stars == 1 ? WANTED_STARTED : WANTED_NOTHING;
}

int wanted_pursuers(const wanted_t* wanted) {
  return wanted->stars == 0 ? 0 : 1 + (wanted->stars - 1) / 2;
}

wanted_event_t wanted_step(wanted_t* wanted, float nearest_police,
                           float player_speed, float dt) {
  wanted->since_crime += dt;
  if (wanted->stars == 0) return WANTED_NOTHING;
  if (nearest_police < 9.0f && player_speed < 4.0f) {
    wanted->caught_for += dt;
    if (wanted->caught_for >= 2.0f) {
      wanted->stars = 0;
      wanted->caught_for = 0;
      return WANTED_BUSTED;
    }
  } else {
    wanted->caught_for = 0;
  }
  wanted->chase_time += dt;
  if (nearest_police < 150.0f) wanted->engaged = 1;
  if (nearest_police > 220.0f && wanted->engaged)
    wanted->out_of_reach += dt;
  else if (nearest_police <= 220.0f)
    wanted->out_of_reach = 0;
  if (wanted->out_of_reach >= 12.0f || wanted->chase_time >= 90.0f) {
    wanted->stars = 0;
    wanted->out_of_reach = 0;
    wanted->caught_for = 0;
    return WANTED_ESCAPED;
  }
  return WANTED_NOTHING;
}
