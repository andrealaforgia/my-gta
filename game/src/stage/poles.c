#include "poles.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define FALL_SECONDS_BASE 0.5f
#define FALL_ANGLE 1.45f /* radians from upright to lying, a touch short */

static float fall_duration(const pole_t* pole) {
  return FALL_SECONDS_BASE + 0.06f * pole->height;
}

/* Lamp and signal lights stop working with the pole: their entities no
 * longer carry the names the stage lights by. */
static void put_out_lights(world_t* world, const pole_t* pole) {
  for (int i = 0; i < pole->entity_count; i++) {
    entity_t* entity = world_entity(world, pole->entities[i]);
    if (!entity) continue;
    if (strncmp(entity->label, "lamp_light", 10) == 0 ||
        strncmp(entity->label, "signal_", 7) == 0) {
      char dark[ENTITY_LABEL_LENGTH];
      snprintf(dark, sizeof(dark), "dead %s", entity->label);
      memcpy(entity->label, dark, sizeof(dark));
      entity->material.emissive = 0.0f;
    }
  }
}

static void topple(scene_t* scene, physics_world_t* physics, world_t* world,
                   pole_t* pole, vec3_t direction) {
  physics_remove_collider(physics, pole->collider);
  pole->collider.id = 0;
  pole->state = POLE_FALLING;
  pole->fall = 0;
  pole->pushed = direction;
  put_out_lights(world, pole);
  /* A signal that is down no longer governs anyone. */
  if (pole->kind != POLE_SIGNAL) return;
  for (size_t i = 0; i < scene->signal_count; i++) {
    float dx = scene->signals[i].x - pole->foot.x;
    float dz = scene->signals[i].z - pole->foot.z;
    if (dx * dx + dz * dz < 2.5f * 2.5f) scene->signals[i].group = -1;
  }
}

int poles_topple_struck(scene_t* scene, physics_world_t* physics,
                        world_t* world, body_handle_t car, vec3_t velocity) {
  vec3_t flat = vec3(velocity.x, 0, velocity.z);
  float speed = vec3_length(flat);
  if (speed < POLE_TOPPLE_SPEED || scene->pole_count == 0) return 0;
  vec3_t direction = vec3_scale(flat, 1.0f / speed);
  int toppled = 0;
  for (size_t c = 0; c < physics_contact_count(physics); c++) {
    const contact_event_t* contact = physics_contact_at(physics, c);
    if (contact->body.id != car.id || contact->collider.id == 0) continue;
    for (size_t i = 0; i < scene->pole_count; i++) {
      pole_t* pole = &scene->poles[i];
      if (pole->state != POLE_STANDING ||
          pole->collider.id != contact->collider.id)
        continue;
      topple(scene, physics, world, pole, direction);
      toppled++;
      break;
    }
  }
  return toppled;
}

void poles_step(scene_t* scene, world_t* world, float dt) {
  for (size_t i = 0; i < scene->pole_count; i++) {
    pole_t* pole = &scene->poles[i];
    if (pole->state != POLE_FALLING) continue;
    pole->fall += dt;
    float t = fminf(pole->fall / fall_duration(pole), 1.0f);
    /* Slow to start, then it picks up speed as it goes over. */
    float angle = FALL_ANGLE * t * t;
    vec3_t axis = vec3(pole->pushed.z, 0, -pole->pushed.x);
    quat_t rotation = quat_from_axis_angle(axis, angle);
    /* Turn about the foot: the meshes are in world coordinates. */
    vec3_t turned = quat_rotate(rotation, pole->foot);
    transform_t transform =
        transform_at(vec3_sub(pole->foot, turned), rotation);
    for (int e = 0; e < pole->entity_count; e++)
      world_set_transform(world, pole->entities[e], transform);
    if (t >= 1.0f) pole->state = POLE_DOWN;
  }
}

size_t poles_standing(const scene_t* scene) {
  size_t standing = 0;
  for (size_t i = 0; i < scene->pole_count; i++)
    standing += scene->poles[i].state == POLE_STANDING;
  return standing;
}
