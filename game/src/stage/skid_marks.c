#include "skid_marks.h"

#include <math.h>
#include <string.h>

void skid_marks_reset(skid_marks_t* marks) { memset(marks, 0, sizeof(*marks)); }

void skid_marks_wheel(skid_marks_t* marks, int wheel, vec3_t contact,
                      bool sliding) {
  if (wheel < 0 || wheel >= SKID_MARK_WHEELS) return;
  if (sliding && marks->sliding[wheel]) {
    float distance = vec3_distance(contact, marks->last[wheel]);
    if (distance > 0.08f && distance < 3.0f) {
      skid_mark_t* mark = &marks->marks[marks->next];
      mark->from = marks->last[wheel];
      mark->to = contact;
      marks->next = (marks->next + 1) % SKID_MARK_CAPACITY;
      if (marks->count < SKID_MARK_CAPACITY) marks->count++;
      marks->last[wheel] = contact;
    } else if (distance >= 3.0f) {
      marks->last[wheel] = contact; /* a jump: start afresh */
    }
    /* Too short a hop: wait until the wheel has moved on. */
  } else {
    marks->last[wheel] = contact;
  }
  marks->sliding[wheel] = sliding;
}

mesh_handle_t skid_mark_mesh_create(renderer_t* renderer) {
  mesh_data_t mesh = mesh_data_create();
  mesh_data_add_quad(&mesh, vec3(-0.5f, 0, 1), vec3(0.5f, 0, 1),
                     vec3(0.5f, 0, 0), vec3(-0.5f, 0, 0), 1);
  mesh_handle_t handle = renderer_upload_mesh(renderer, &mesh);
  mesh_data_free(&mesh);
  return handle;
}

int skid_marks_draw(const skid_marks_t* marks, mesh_handle_t strip,
                    renderer_t* renderer, vec3_t eye, float range) {
  material_t ink = material_rough(vec3(0.025f, 0.025f, 0.027f), 0.95f);
  int drawn = 0;
  for (size_t i = 0; i < marks->count; i++) {
    const skid_mark_t* mark = &marks->marks[i];
    float dx = mark->to.x - mark->from.x, dz = mark->to.z - mark->from.z;
    float length = sqrtf(dx * dx + dz * dz);
    if (length < 0.01f) continue;
    float ex = mark->from.x - eye.x, ez = mark->from.z - eye.z;
    if (ex * ex + ez * ez > range * range) continue;
    float yaw = atan2f(dx, dz);
    vec3_t origin = vec3_add(mark->from, vec3(0, 0.012f, 0));
    /* Lengthen a hair so neighbouring segments meet. */
    mat4_t model = mat4_multiply(
        mat4_translation(origin),
        mat4_multiply(mat4_from_quat(quat_from_yaw(yaw + 3.14159265f)),
                      mat4_scaling(vec3(0.22f, 1.0f, length * 1.05f))));
    renderer_draw_mesh(renderer, strip, ink, model);
    drawn++;
  }
  return drawn;
}
