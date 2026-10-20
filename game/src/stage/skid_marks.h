/**
 * @file skid_marks.h
 * @brief Black tyre marks left on the road where a wheel slides: a ring
 * of short segments joining where each wheel touched the ground a moment
 * before and now, drawn as dark strips.
 */
#ifndef GAME_SRC_STAGE_SKID_MARKS_H_
#define GAME_SRC_STAGE_SKID_MARKS_H_

#include <stdbool.h>
#include <stddef.h>

#include "renderer.h"

#define SKID_MARK_CAPACITY 600
#define SKID_MARK_WHEELS 4

typedef struct {
  vec3_t from, to;
} skid_mark_t;

typedef struct {
  skid_mark_t marks[SKID_MARK_CAPACITY];
  size_t count; /* marks held, up to the capacity */
  size_t next;  /* where the next one goes */
  vec3_t last[SKID_MARK_WHEELS];
  bool sliding[SKID_MARK_WHEELS]; /* was marking at the last contact */
} skid_marks_t;

void skid_marks_reset(skid_marks_t* marks);
/* Tell it where wheel `wheel` touches the road and whether it is sliding. A
 * mark is laid from the last contact if both were sliding and the wheel
 * moved a sensible distance. */
void skid_marks_wheel(skid_marks_t* marks, int wheel, vec3_t contact,
                      bool sliding);
/* A one-metre strip lying along +z, for drawing the marks. */
mesh_handle_t skid_mark_mesh_create(renderer_t* renderer);
/* Draws the marks within `range` of the eye; returns how many. */
int skid_marks_draw(const skid_marks_t* marks, mesh_handle_t strip,
                    renderer_t* renderer, vec3_t eye, float range);

#endif  // GAME_SRC_STAGE_SKID_MARKS_H_
