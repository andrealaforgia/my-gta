/**
 * @file pedestrians.h
 * @brief People walking the pavements. They follow both sides of every
 * road that has a pavement, and the middle of pedestrian streets and
 * paths, pick a way at each junction, turn back at dead ends, and stop
 * when the player's car is close. Adults, children, elders and joggers
 * differ in size and pace, some walk in pairs, and they are drawn as
 * simple figures with swinging arms and legs, hair, skirts, bags and caps;
 * they are not solid.
 */
#ifndef GAME_SRC_TRAFFIC_PEDESTRIANS_H_
#define GAME_SRC_TRAFFIC_PEDESTRIANS_H_

#include <stddef.h>

#include "city_package.h"
#include "physics_world.h"
#include "renderer.h"

typedef struct pedestrians pedestrians_t;

typedef struct {
  vec3_t position; /* feet */
  float yaw;
  float speed;
  float phase; /* walking cycle in radians */
  int outfit;
  int route;
  float fall;  /* 0 upright, 1 flat on the ground, more while somersaulting */
  float lift;  /* metres off the ground while thrown through the air */
  float flail; /* 0 calm, 1 arms and legs thrashing, after a hit */
} pedestrian_t;

typedef enum {
  PERSON_ADULT,
  PERSON_CHILD, /* smaller, a little quicker */
  PERSON_ELDER, /* slower, in a cap */
  PERSON_JOGGER /* fast, arms pumping */
} person_kind_t;

/* What sort of person an outfit number makes, and how tall (1 is an adult). */
person_kind_t pedestrian_kind(int outfit);
float pedestrian_scale(int outfit);

typedef struct {
  mesh_handle_t torso, head, leg, arm, skirt, hair_short, hair_long, bag,
      backpack, cap, face;
  texture_handle_t cloth, skin, hair; /* none until given */
} pedestrian_meshes_t;

pedestrians_t* pedestrians_create(const city_route_t* routes,
                                  size_t route_count, physics_world_t* physics,
                                  int count, uint32_t seed);
void pedestrians_destroy(pedestrians_t* pedestrians);
void pedestrians_step(pedestrians_t* pedestrians, vec3_t player_position,
                      float dt);
size_t pedestrians_count(const pedestrians_t* pedestrians);
pedestrian_t pedestrians_get(const pedestrians_t* pedestrians, size_t index);
/* The place people should be thick around, usually the player: someone more
 * than 700 m from it is moved to a pavement 100 to 450 m from it. */
void pedestrians_set_focus(pedestrians_t* pedestrians, vec3_t focus);
/* Zebra crossings (x, z pairs, borrowed): people walking a pavement past one
 * sometimes cross there, when the player's car is not close. */
void pedestrians_set_crossings(pedestrians_t* pedestrians, const float* xz,
                               size_t count);
/* The positions of the people who are in the road now, crossing, up to
 * `max`; returns how many. Cars give way to them. */
size_t pedestrians_crossing(const pedestrians_t* pedestrians, vec3_t* out,
                            size_t max);
/* A car at `centre` heading `yaw` (0 faces -z) with `half_extents` and moving
 * at `velocity` runs into whoever stands in its footprint, if it is going
 * faster than a walk: they are thrown ahead of it and lie on the ground.
 * Returns how many were hit. */
int pedestrians_hit(pedestrians_t* pedestrians, vec3_t centre, float yaw,
                    vec3_t half_extents, vec3_t velocity);
/* Something frightening happens at a place: everyone standing within
 * `radius` metres runs, at two and a half times their pace, for a few
 * seconds. */
void pedestrians_alarm(pedestrians_t* pedestrians, vec3_t at, float radius);
/* A bullet from `origin` along `direction` (unit) over `range` metres hits
 * the nearest person standing in its way, who goes down as if run over.
 * Returns true, with the distance in `hit_distance`, if it hit anyone. */
bool pedestrians_shoot(pedestrians_t* pedestrians, vec3_t origin,
                       vec3_t direction, float range, float* hit_distance);
/* Stands someone on a route: side +1 is the right of its direction. */
void pedestrians_place(pedestrians_t* pedestrians, size_t index, int route,
                       int side, int direction, float distance);

pedestrian_meshes_t pedestrian_meshes_create(renderer_t* renderer);
/* Dresses the figures in textures: cloth for garments, skin, hair. */
void pedestrian_meshes_set_textures(pedestrian_meshes_t* meshes,
                                    texture_handle_t cloth,
                                    texture_handle_t skin,
                                    texture_handle_t hair);
void pedestrian_meshes_destroy(pedestrian_meshes_t* meshes,
                               renderer_t* renderer);
/* One figure at a place (feet), facing yaw, as people walking are drawn: the
 * outfit number chooses clothes and build, speed and phase the stride. */
void pedestrian_draw_figure(const pedestrian_meshes_t* meshes,
                            renderer_t* renderer, vec3_t position, float yaw,
                            float speed, float phase, int outfit);
/* The same, tipped over backwards by `fall` quarter turns (1 is lying) and
 * `lift` metres in the air, arms and legs thrashing by `flail` (0 to 1), the
 * right arm thrown forward in a punch by `punch` (0 to 1). */
void pedestrian_draw_figure_fallen(const pedestrian_meshes_t* meshes,
                                   renderer_t* renderer, vec3_t position,
                                   float yaw, float speed, float phase,
                                   int outfit, float fall, float lift,
                                   float flail, float punch);
/* A punch from `from` (feet) towards `direction` (flat, a unit vector): the
 * nearest person standing within `reach` metres and about sixty degrees of it
 * staggers and then runs away from the puncher; people close by take fright
 * too. Returns how many were hit. */
int pedestrians_punch(pedestrians_t* pedestrians, vec3_t from, vec3_t direction,
                      float reach);

/* People are drawn within this many metres, and never put down nearer than
 * it, so nobody appears in front of the player. */
#define PEDESTRIAN_DRAW_RANGE 200.0f

int pedestrians_draw(const pedestrians_t* pedestrians,
                     const pedestrian_meshes_t* meshes, renderer_t* renderer,
                     vec3_t eye, float range);

#endif  // GAME_SRC_TRAFFIC_PEDESTRIANS_H_
