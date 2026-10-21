#include "mesh_data.h"

#include <math.h>
#include <stdlib.h>

mesh_data_t mesh_data_create(void) {
  mesh_data_t mesh = {NULL, 0, 0, NULL, 0, 0};
  return mesh;
}

void mesh_data_free(mesh_data_t* mesh) {
  free(mesh->vertices);
  free(mesh->indices);
  *mesh = mesh_data_create();
}

static bool grow(void** items, size_t* capacity, size_t needed,
                 size_t item_size) {
  if (needed <= *capacity) return true;
  size_t capacity_wanted = *capacity ? *capacity : 64;
  while (capacity_wanted < needed) capacity_wanted *= 2;
  void* grown = realloc(*items, capacity_wanted * item_size);
  if (!grown) return false;
  *items = grown;
  *capacity = capacity_wanted;
  return true;
}

bool mesh_data_add_vertex(mesh_data_t* mesh, vertex_t vertex) {
  if (!grow((void**)&mesh->vertices, &mesh->vertex_capacity,
            mesh->vertex_count + 1, sizeof(vertex_t)))
    return false;
  mesh->vertices[mesh->vertex_count++] = vertex;
  return true;
}

bool mesh_data_add_triangle(mesh_data_t* mesh, uint32_t a, uint32_t b,
                            uint32_t c) {
  if (!grow((void**)&mesh->indices, &mesh->index_capacity,
            mesh->index_count + 3, sizeof(uint32_t)))
    return false;
  mesh->indices[mesh->index_count++] = a;
  mesh->indices[mesh->index_count++] = b;
  mesh->indices[mesh->index_count++] = c;
  return true;
}

static vertex_t vertex(vec3_t position, vec3_t normal, float u, float v) {
  vertex_t result = {position, normal, u, v};
  return result;
}

bool mesh_data_add_quad(mesh_data_t* mesh, vec3_t a, vec3_t b, vec3_t c,
                        vec3_t d, float uv_scale) {
  vec3_t normal = vec3_normalize(vec3_cross(vec3_sub(b, a), vec3_sub(d, a)));
  /* Planar uv: project onto the two axes most aligned with the quad. */
  vec3_t u_axis = vec3_normalize(vec3_sub(b, a));
  vec3_t v_axis = vec3_cross(normal, u_axis);
  uint32_t base = (uint32_t)mesh->vertex_count;
  vec3_t corners[4] = {a, b, c, d};
  for (int i = 0; i < 4; i++) {
    vec3_t relative = vec3_sub(corners[i], a);
    float u = vec3_dot(relative, u_axis) * uv_scale;
    float v = vec3_dot(relative, v_axis) * uv_scale;
    if (!mesh_data_add_vertex(mesh, vertex(corners[i], normal, u, v)))
      return false;
  }
  return mesh_data_add_triangle(mesh, base, base + 1, base + 2) &&
         mesh_data_add_triangle(mesh, base, base + 2, base + 3);
}

bool mesh_data_add_box(mesh_data_t* mesh, vec3_t c, vec3_t h) {
  vec3_t p[8];
  for (int i = 0; i < 8; i++)
    p[i] = vec3(c.x + ((i & 1) ? h.x : -h.x), c.y + ((i & 2) ? h.y : -h.y),
                c.z + ((i & 4) ? h.z : -h.z));
  /* Faces anticlockwise seen from outside. */
  return mesh_data_add_quad(mesh, p[4], p[5], p[7], p[6], 1) && /* +Z (south) */
         mesh_data_add_quad(mesh, p[1], p[0], p[2], p[3], 1) && /* -Z (north) */
         mesh_data_add_quad(mesh, p[5], p[1], p[3], p[7], 1) && /* +X */
         mesh_data_add_quad(mesh, p[0], p[4], p[6], p[2], 1) && /* -X */
         mesh_data_add_quad(mesh, p[2], p[6], p[7], p[3], 1) && /* +Y */
         mesh_data_add_quad(mesh, p[0], p[1], p[5], p[4], 1);   /* -Y */
}

bool mesh_data_add_cylinder_x(mesh_data_t* mesh, vec3_t center, float radius,
                              float width, int segments) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  float half = width * 0.5f;
  /* Rim vertices: for each segment, one on each side, plus cap centres. */
  for (int i = 0; i < segments; i++) {
    float angle = (float)i / (float)segments * 2.0f * (float)M_PI;
    vec3_t radial = vec3(0, cosf(angle), sinf(angle));
    vec3_t left = vec3_add(
        center, vec3_add(vec3_scale(radial, radius), vec3(-half, 0, 0)));
    vec3_t right = vec3_add(
        center, vec3_add(vec3_scale(radial, radius), vec3(half, 0, 0)));
    float u = (float)i / (float)segments;
    if (!mesh_data_add_vertex(mesh, vertex(left, radial, u, 0))) return false;
    if (!mesh_data_add_vertex(mesh, vertex(right, radial, u, 1))) return false;
  }
  for (int i = 0; i < segments; i++) {
    uint32_t next = (uint32_t)((i + 1) % segments);
    uint32_t l0 = base + (uint32_t)i * 2, r0 = l0 + 1;
    uint32_t l1 = base + next * 2, r1 = l1 + 1;
    if (!mesh_data_add_triangle(mesh, l0, l1, r1)) return false;
    if (!mesh_data_add_triangle(mesh, l0, r1, r0)) return false;
  }
  /* Caps as fans around a centre vertex. */
  for (int side = 0; side < 2; side++) {
    float x = side ? half : -half;
    vec3_t normal = vec3(side ? 1.0f : -1.0f, 0, 0);
    uint32_t centre_index = (uint32_t)mesh->vertex_count;
    if (!mesh_data_add_vertex(
            mesh, vertex(vec3_add(center, vec3(x, 0, 0)), normal, 0.5f, 0.5f)))
      return false;
    uint32_t rim = (uint32_t)mesh->vertex_count;
    for (int i = 0; i < segments; i++) {
      float angle = (float)i / (float)segments * 2.0f * (float)M_PI;
      vec3_t p =
          vec3_add(center, vec3(x, cosf(angle) * radius, sinf(angle) * radius));
      if (!mesh_data_add_vertex(mesh,
                                vertex(p, normal, 0.5f + cosf(angle) * 0.5f,
                                       0.5f + sinf(angle) * 0.5f)))
        return false;
    }
    for (int i = 0; i < segments; i++) {
      uint32_t a = rim + (uint32_t)i, b = rim + (uint32_t)((i + 1) % segments);
      bool ok = side ? mesh_data_add_triangle(mesh, centre_index, a, b)
                     : mesh_data_add_triangle(mesh, centre_index, b, a);
      if (!ok) return false;
    }
  }
  return true;
}

bool mesh_data_add_grid(mesh_data_t* mesh, float half_size, float spacing) {
  int cells = (int)(half_size * 2 / spacing);
  for (int z = 0; z < cells; z++) {
    for (int x = 0; x < cells; x++) {
      float x0 = -half_size + x * spacing, x1 = x0 + spacing;
      float z0 = -half_size + z * spacing, z1 = z0 + spacing;
      if (!mesh_data_add_quad(mesh, vec3(x0, 0, z1), vec3(x1, 0, z1),
                              vec3(x1, 0, z0), vec3(x0, 0, z0), 1.0f / spacing))
        return false;
    }
  }
  return true;
}

bool mesh_data_append(mesh_data_t* mesh, const mesh_data_t* other) {
  uint32_t base = (uint32_t)mesh->vertex_count;
  for (size_t i = 0; i < other->vertex_count; i++)
    if (!mesh_data_add_vertex(mesh, other->vertices[i])) return false;
  for (size_t i = 0; i + 2 < other->index_count; i += 3)
    if (!mesh_data_add_triangle(mesh, base + other->indices[i],
                                base + other->indices[i + 1],
                                base + other->indices[i + 2]))
      return false;
  return true;
}

aabb_t mesh_data_bounds(const mesh_data_t* mesh) {
  aabb_t box = aabb_empty();
  for (size_t i = 0; i < mesh->vertex_count; i++)
    box = aabb_include(box, mesh->vertices[i].position);
  return box;
}

size_t mesh_data_triangle_count(const mesh_data_t* mesh) {
  return mesh->index_count / 3;
}

static vertex_t midpoint(vertex_t a, vertex_t b) {
  vertex_t m;
  m.position = vec3_scale(vec3_add(a.position, b.position), 0.5f);
  m.normal = vec3_normalize(vec3_add(a.normal, b.normal));
  m.u = 0.5f * (a.u + b.u);
  m.v = 0.5f * (a.v + b.v);
  return m;
}

/* Splits one triangle recursively into `out`. Depth is bounded so a huge
 * triangle cannot run away. */
static bool subdivide_triangle(mesh_data_t* out, vertex_t a, vertex_t b,
                               vertex_t c, float max_edge, int depth) {
  float ab = vec3_distance(a.position, b.position);
  float bc = vec3_distance(b.position, c.position);
  float ca = vec3_distance(c.position, a.position);
  float longest = fmaxf(ab, fmaxf(bc, ca));
  if (longest <= max_edge || depth >= 12) {
    uint32_t base = (uint32_t)out->vertex_count;
    return mesh_data_add_vertex(out, a) && mesh_data_add_vertex(out, b) &&
           mesh_data_add_vertex(out, c) &&
           mesh_data_add_triangle(out, base, base + 1, base + 2);
  }
  if (ab >= bc && ab >= ca) {
    vertex_t m = midpoint(a, b);
    return subdivide_triangle(out, a, m, c, max_edge, depth + 1) &&
           subdivide_triangle(out, m, b, c, max_edge, depth + 1);
  }
  if (bc >= ca) {
    vertex_t m = midpoint(b, c);
    return subdivide_triangle(out, a, b, m, max_edge, depth + 1) &&
           subdivide_triangle(out, a, m, c, max_edge, depth + 1);
  }
  vertex_t m = midpoint(c, a);
  return subdivide_triangle(out, a, b, m, max_edge, depth + 1) &&
         subdivide_triangle(out, m, b, c, max_edge, depth + 1);
}

bool mesh_data_subdivide(mesh_data_t* mesh, float max_edge) {
  mesh_data_t out = mesh_data_create();
  for (size_t i = 0; i + 2 < mesh->index_count; i += 3) {
    if (!subdivide_triangle(&out, mesh->vertices[mesh->indices[i]],
                            mesh->vertices[mesh->indices[i + 1]],
                            mesh->vertices[mesh->indices[i + 2]], max_edge,
                            0)) {
      mesh_data_free(&out);
      return false;
    }
  }
  mesh_data_free(mesh);
  *mesh = out;
  return true;
}

bool mesh_data_add_ellipsoid(mesh_data_t* mesh, vec3_t center, vec3_t radii,
                             int rings, int segments) {
  if (rings < 2 || segments < 3) return false;
  uint32_t base = (uint32_t)mesh->vertex_count;
  for (int r = 0; r <= rings; r++) {
    float phi = (float)M_PI * (float)r / (float)rings; /* 0 at the top pole */
    for (int s = 0; s <= segments; s++) {
      float theta = 2.0f * (float)M_PI * (float)s / (float)segments;
      vec3_t unit =
          vec3(sinf(phi) * cosf(theta), cosf(phi), sinf(phi) * sinf(theta));
      vertex_t v;
      v.position =
          vec3(center.x + unit.x * radii.x, center.y + unit.y * radii.y,
               center.z + unit.z * radii.z);
      /* The normal of an ellipsoid is the unit point over the radii. */
      v.normal = vec3_normalize(
          vec3(unit.x / radii.x, unit.y / radii.y, unit.z / radii.z));
      v.u = (float)s / (float)segments;
      v.v = (float)r / (float)rings;
      if (!mesh_data_add_vertex(mesh, v)) return false;
    }
  }
  uint32_t row = (uint32_t)segments + 1;
  for (int r = 0; r < rings; r++) {
    for (int s = 0; s < segments; s++) {
      uint32_t a = base + (uint32_t)r * row + (uint32_t)s;
      uint32_t b = a + 1, c = a + row, d = c + 1;
      /* Anticlockwise seen from outside: round, then down. */
      if (r > 0 && !mesh_data_add_triangle(mesh, a, b, c)) return false;
      if (r < rings - 1 && !mesh_data_add_triangle(mesh, b, d, c)) return false;
    }
  }
  return true;
}

bool mesh_data_add_cylinder_y(mesh_data_t* mesh, vec3_t base, float radius,
                              float height, int segments) {
  if (segments < 3) return false;
  uint32_t start = (uint32_t)mesh->vertex_count;
  for (int s = 0; s <= segments; s++) {
    float theta = 2.0f * (float)M_PI * (float)s / (float)segments;
    vec3_t out = vec3(cosf(theta), 0, sinf(theta));
    vertex_t bottom = {
        vec3(base.x + out.x * radius, base.y, base.z + out.z * radius), out,
        (float)s / (float)segments, 0};
    vertex_t top = {
        vec3(base.x + out.x * radius, base.y + height, base.z + out.z * radius),
        out, (float)s / (float)segments, 1};
    if (!mesh_data_add_vertex(mesh, bottom) || !mesh_data_add_vertex(mesh, top))
      return false;
  }
  for (int s = 0; s < segments; s++) {
    uint32_t a = start + (uint32_t)s * 2, b = a + 1, c = a + 2, d = a + 3;
    /* Outward faces anticlockwise from outside: bottom a, then c, then top. */
    if (!mesh_data_add_triangle(mesh, a, b, c) ||
        !mesh_data_add_triangle(mesh, c, b, d))
      return false;
  }
  for (int cap = 0; cap < 2; cap++) {
    float y = base.y + (cap ? height : 0.0f);
    vec3_t normal = vec3(0, cap ? 1.0f : -1.0f, 0);
    uint32_t centre_index = (uint32_t)mesh->vertex_count;
    vertex_t middle = {vec3(base.x, y, base.z), normal, 0.5f, 0.5f};
    if (!mesh_data_add_vertex(mesh, middle)) return false;
    uint32_t ring = (uint32_t)mesh->vertex_count;
    for (int s = 0; s < segments; s++) {
      float theta = 2.0f * (float)M_PI * (float)s / (float)segments;
      vertex_t v = {
          vec3(base.x + cosf(theta) * radius, y, base.z + sinf(theta) * radius),
          normal, 0, 0};
      if (!mesh_data_add_vertex(mesh, v)) return false;
    }
    for (int s = 0; s < segments; s++) {
      uint32_t a = ring + (uint32_t)s,
               b = ring + (uint32_t)((s + 1) % segments);
      if (!(cap ? mesh_data_add_triangle(mesh, centre_index, b, a)
                : mesh_data_add_triangle(mesh, centre_index, a, b)))
        return false;
    }
  }
  return true;
}

size_t mesh_data_remove_degenerate(mesh_data_t* mesh, float epsilon) {
  size_t kept = 0, removed = 0;
  for (size_t i = 0; i + 2 < mesh->index_count; i += 3) {
    vec3_t a = mesh->vertices[mesh->indices[i]].position;
    vec3_t b = mesh->vertices[mesh->indices[i + 1]].position;
    vec3_t c = mesh->vertices[mesh->indices[i + 2]].position;
    if (vec3_length(vec3_cross(vec3_sub(b, a), vec3_sub(c, a))) <
        2.0f * epsilon) {
      removed++;
      continue;
    }
    if (kept != i) {
      mesh->indices[kept] = mesh->indices[i];
      mesh->indices[kept + 1] = mesh->indices[i + 1];
      mesh->indices[kept + 2] = mesh->indices[i + 2];
    }
    kept += 3;
  }
  mesh->index_count = kept;
  return removed;
}
