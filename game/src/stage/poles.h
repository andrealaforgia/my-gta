/**
 * @file poles.h
 * @brief Lamp posts, signals and road signs that go over when the car hits
 * them hard enough: the collider goes at once, the pole topples away from
 * the car in a second or so and lies on the ground, and the lamp and signal
 * lights go dark.
 */
#ifndef GAME_SRC_STAGE_POLES_H_
#define GAME_SRC_STAGE_POLES_H_

#include "physics_world.h"
#include "scene.h"
#include "world.h"

/* A car must be going at least this fast, in metres per second, to knock a
 * pole down; slower, it bumps against it. */
#define POLE_TOPPLE_SPEED 2.5f

/* Looks at the contacts the car made in the last physics step and topples
 * the poles it struck at `speed_before` along `direction`. Returns how
 * many went down. */
int poles_topple_struck(scene_t* scene, physics_world_t* physics,
                        world_t* world, body_handle_t car, vec3_t velocity);
/* Tips the falling poles over. */
void poles_step(scene_t* scene, world_t* world, float dt);
/* How many poles are still standing. */
size_t poles_standing(const scene_t* scene);

#endif  // GAME_SRC_STAGE_POLES_H_
