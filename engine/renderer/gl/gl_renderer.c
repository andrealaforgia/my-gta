/* OpenGL 4.1 core backend for renderer.h. The only file that includes GL. */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#endif

#include "font5x7.h"
#include "log.h"
#include "renderer.h"

#define MAX_MESHES 4096
#define MAX_TEXTURES 256
#define MAX_LINE_VERTICES (1 << 16)
#define MAX_TEXT_VERTICES (1 << 17)

typedef struct {
  GLuint vao, vbo, ebo;
  GLsizei index_count;
  bool used;
} gl_mesh_t;

typedef struct {
  vec3_t position;
  vec3_t color;
} line_vertex_t;

typedef struct {
  float x, y;
  vec3_t color;
} text_vertex_t;

struct renderer {
  int width, height;
  GLuint mesh_program, line_program, text_program;
  gl_mesh_t meshes[MAX_MESHES];
  GLuint textures[MAX_TEXTURES];
  GLuint white_texture;
  GLuint line_vao, line_vbo;
  line_vertex_t* line_vertices;
  size_t line_count;
  GLuint text_vao, text_vbo;
  text_vertex_t* text_vertices;
  size_t text_count;
  mat4_t view, projection, view_projection;
  vec3_t eye;
  render_environment_t environment;
  render_stats_t stats;
  int meshes_uploaded;
};

static const char* mesh_vertex_source =
    "#version 410 core\n"
    "layout(location = 0) in vec3 in_position;\n"
    "layout(location = 1) in vec3 in_normal;\n"
    "layout(location = 2) in vec2 in_uv;\n"
    "uniform mat4 u_model;\n"
    "uniform mat4 u_view_projection;\n"
    "out vec3 v_world_position;\n"
    "out vec3 v_normal;\n"
    "out vec2 v_uv;\n"
    "void main() {\n"
    "  vec4 world = u_model * vec4(in_position, 1.0);\n"
    "  v_world_position = world.xyz;\n"
    "  v_normal = mat3(u_model) * in_normal;\n"
    "  v_uv = in_uv;\n"
    "  gl_Position = u_view_projection * world;\n"
    "}\n";

static const char* mesh_fragment_source =
    "#version 410 core\n"
    "in vec3 v_world_position;\n"
    "in vec3 v_normal;\n"
    "in vec2 v_uv;\n"
    "uniform vec3 u_base_color;\n"
    "uniform float u_metallic;\n"
    "uniform float u_roughness;\n"
    "uniform int u_has_texture;\n"
    "uniform sampler2D u_texture;\n"
    "uniform vec3 u_light_direction;\n"
    "uniform vec3 u_light_color;\n"
    "uniform float u_ambient;\n"
    "uniform vec3 u_eye;\n"
    "uniform vec3 u_sky_color;\n"
    "uniform float u_fog_start;\n"
    "uniform float u_fog_end;\n"
    "out vec4 out_color;\n"
    "void main() {\n"
    "  vec3 albedo = u_base_color;\n"
    "  if (u_has_texture == 1) albedo *= texture(u_texture, v_uv).rgb;\n"
    "  vec3 n = normalize(v_normal);\n"
    "  vec3 l = normalize(u_light_direction);\n"
    "  vec3 v = normalize(u_eye - v_world_position);\n"
    "  vec3 h = normalize(l + v);\n"
    "  float n_dot_l = max(dot(n, l), 0.0);\n"
    "  float shininess = mix(256.0, 4.0, u_roughness);\n"
    "  float spec = pow(max(dot(n, h), 0.0), shininess) * (1.0 - u_roughness * "
    "0.7);\n"
    "  vec3 specular_tint = mix(vec3(0.04), albedo, u_metallic);\n"
    "  vec3 diffuse = albedo * (1.0 - u_metallic);\n"
    "  vec3 hemisphere = mix(albedo * 0.35, albedo, n.y * 0.5 + 0.5);\n"
    "  vec3 color = hemisphere * u_ambient\n"
    "             + (diffuse * n_dot_l + specular_tint * spec * n_dot_l) * "
    "u_light_color;\n"
    "  float distance = length(u_eye - v_world_position);\n"
    "  float fog = clamp((distance - u_fog_start) / (u_fog_end - u_fog_start), "
    "0.0, 1.0);\n"
    "  color = mix(color, u_sky_color, fog);\n"
    "  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);\n"
    "}\n";

static const char* line_vertex_source =
    "#version 410 core\n"
    "layout(location = 0) in vec3 in_position;\n"
    "layout(location = 1) in vec3 in_color;\n"
    "uniform mat4 u_view_projection;\n"
    "out vec3 v_color;\n"
    "void main() { v_color = in_color; gl_Position = u_view_projection * "
    "vec4(in_position, 1.0); }\n";

static const char* line_fragment_source =
    "#version 410 core\n"
    "in vec3 v_color;\n"
    "out vec4 out_color;\n"
    "void main() { out_color = vec4(v_color, 1.0); }\n";

static const char* text_vertex_source =
    "#version 410 core\n"
    "layout(location = 0) in vec2 in_position;\n"
    "layout(location = 1) in vec3 in_color;\n"
    "uniform vec2 u_viewport;\n"
    "out vec3 v_color;\n"
    "void main() {\n"
    "  v_color = in_color;\n"
    "  vec2 ndc = vec2(in_position.x / u_viewport.x * 2.0 - 1.0, 1.0 - "
    "in_position.y / u_viewport.y * 2.0);\n"
    "  gl_Position = vec4(ndc, 0.0, 1.0);\n"
    "}\n";

static GLuint compile_shader(GLenum type, const char* source,
                             const char* name) {
  GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, NULL);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char message[2048];
    glGetShaderInfoLog(shader, sizeof(message), NULL, message);
    LOG_ERROR("renderer", "shader %s failed to compile: %s", name, message);
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

static GLuint link_program(const char* vertex, const char* fragment,
                           const char* name) {
  GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex, name);
  GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment, name);
  if (!vs || !fs) return 0;
  GLuint program = glCreateProgram();
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);
  glDeleteShader(vs);
  glDeleteShader(fs);
  GLint ok = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &ok);
  if (!ok) {
    char message[2048];
    glGetProgramInfoLog(program, sizeof(message), NULL, message);
    LOG_ERROR("renderer", "program %s failed to link: %s", name, message);
    glDeleteProgram(program);
    return 0;
  }
  return program;
}

static void check_gl(const char* where) {
  GLenum error = glGetError();
  if (error != GL_NO_ERROR)
    LOG_ERROR("renderer", "GL error 0x%x at %s", error, where);
}

renderer_t* renderer_create(int drawable_width, int drawable_height) {
  renderer_t* r = calloc(1, sizeof(renderer_t));
  if (!r) return NULL;
  r->mesh_program =
      link_program(mesh_vertex_source, mesh_fragment_source, "mesh");
  r->line_program =
      link_program(line_vertex_source, line_fragment_source, "line");
  r->text_program =
      link_program(text_vertex_source, line_fragment_source, "text");
  r->line_vertices = malloc(MAX_LINE_VERTICES * sizeof(line_vertex_t));
  r->text_vertices = malloc(MAX_TEXT_VERTICES * sizeof(text_vertex_t));
  if (!r->mesh_program || !r->line_program || !r->text_program ||
      !r->line_vertices || !r->text_vertices) {
    renderer_destroy(r);
    return NULL;
  }
  glGenVertexArrays(1, &r->line_vao);
  glGenBuffers(1, &r->line_vbo);
  glBindVertexArray(r->line_vao);
  glBindBuffer(GL_ARRAY_BUFFER, r->line_vbo);
  glBufferData(GL_ARRAY_BUFFER, MAX_LINE_VERTICES * sizeof(line_vertex_t), NULL,
               GL_STREAM_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(line_vertex_t),
                        (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(line_vertex_t),
                        (void*)offsetof(line_vertex_t, color));

  glGenVertexArrays(1, &r->text_vao);
  glGenBuffers(1, &r->text_vbo);
  glBindVertexArray(r->text_vao);
  glBindBuffer(GL_ARRAY_BUFFER, r->text_vbo);
  glBufferData(GL_ARRAY_BUFFER, MAX_TEXT_VERTICES * sizeof(text_vertex_t), NULL,
               GL_STREAM_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(text_vertex_t),
                        (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(text_vertex_t),
                        (void*)offsetof(text_vertex_t, color));
  glBindVertexArray(0);

  static const uint8_t white[4] = {255, 255, 255, 255};
  glGenTextures(1, &r->white_texture);
  glBindTexture(GL_TEXTURE_2D, r->white_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE,
               white);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  renderer_resize(r, drawable_width, drawable_height);
  check_gl("renderer_create");
  LOG_INFO("renderer", "OpenGL %s on %s", (const char*)glGetString(GL_VERSION),
           (const char*)glGetString(GL_RENDERER));
  return r;
}

void renderer_destroy(renderer_t* r) {
  if (!r) return;
  for (uint32_t i = 1; i < MAX_MESHES; i++) {
    mesh_handle_t handle = {i};
    if (r->meshes[i].used) renderer_release_mesh(r, handle);
  }
  for (uint32_t i = 1; i < MAX_TEXTURES; i++) {
    texture_handle_t handle = {i};
    if (r->textures[i]) renderer_release_texture(r, handle);
  }
  if (r->white_texture) glDeleteTextures(1, &r->white_texture);
  if (r->line_vbo) glDeleteBuffers(1, &r->line_vbo);
  if (r->line_vao) glDeleteVertexArrays(1, &r->line_vao);
  if (r->text_vbo) glDeleteBuffers(1, &r->text_vbo);
  if (r->text_vao) glDeleteVertexArrays(1, &r->text_vao);
  if (r->mesh_program) glDeleteProgram(r->mesh_program);
  if (r->line_program) glDeleteProgram(r->line_program);
  if (r->text_program) glDeleteProgram(r->text_program);
  free(r->line_vertices);
  free(r->text_vertices);
  free(r);
}

void renderer_resize(renderer_t* r, int drawable_width, int drawable_height) {
  r->width = drawable_width > 0 ? drawable_width : 1;
  r->height = drawable_height > 0 ? drawable_height : 1;
  glViewport(0, 0, r->width, r->height);
}

mesh_handle_t renderer_upload_mesh(renderer_t* r, const mesh_data_t* mesh) {
  mesh_handle_t none = {0};
  if (mesh->vertex_count == 0 || mesh->index_count == 0) {
    LOG_WARN("renderer", "refusing to upload an empty mesh");
    return none;
  }
  uint32_t slot = 0;
  for (uint32_t i = 1; i < MAX_MESHES; i++) {
    if (!r->meshes[i].used) {
      slot = i;
      break;
    }
  }
  if (!slot) {
    LOG_ERROR("renderer", "mesh table full (%d)", MAX_MESHES);
    return none;
  }
  gl_mesh_t* m = &r->meshes[slot];
  glGenVertexArrays(1, &m->vao);
  glGenBuffers(1, &m->vbo);
  glGenBuffers(1, &m->ebo);
  glBindVertexArray(m->vao);
  glBindBuffer(GL_ARRAY_BUFFER, m->vbo);
  glBufferData(GL_ARRAY_BUFFER,
               (GLsizeiptr)(mesh->vertex_count * sizeof(vertex_t)),
               mesh->vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m->ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               (GLsizeiptr)(mesh->index_count * sizeof(uint32_t)),
               mesh->indices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_t), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_t),
                        (void*)offsetof(vertex_t, normal));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(vertex_t),
                        (void*)offsetof(vertex_t, u));
  glBindVertexArray(0);
  m->index_count = (GLsizei)mesh->index_count;
  m->used = true;
  r->meshes_uploaded++;
  check_gl("renderer_upload_mesh");
  mesh_handle_t handle = {slot};
  return handle;
}

void renderer_release_mesh(renderer_t* r, mesh_handle_t handle) {
  if (handle.id == 0 || handle.id >= MAX_MESHES || !r->meshes[handle.id].used)
    return;
  gl_mesh_t* m = &r->meshes[handle.id];
  glDeleteBuffers(1, &m->vbo);
  glDeleteBuffers(1, &m->ebo);
  glDeleteVertexArrays(1, &m->vao);
  memset(m, 0, sizeof(*m));
  r->meshes_uploaded--;
}

texture_handle_t renderer_upload_texture(renderer_t* r, int width, int height,
                                         const uint8_t* rgba) {
  texture_handle_t none = {0};
  uint32_t slot = 0;
  for (uint32_t i = 1; i < MAX_TEXTURES; i++) {
    if (!r->textures[i]) {
      slot = i;
      break;
    }
  }
  if (!slot) {
    LOG_ERROR("renderer", "texture table full (%d)", MAX_TEXTURES);
    return none;
  }
  glGenTextures(1, &r->textures[slot]);
  glBindTexture(GL_TEXTURE_2D, r->textures[slot]);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, rgba);
  glGenerateMipmap(GL_TEXTURE_2D);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  check_gl("renderer_upload_texture");
  texture_handle_t handle = {slot};
  return handle;
}

void renderer_release_texture(renderer_t* r, texture_handle_t handle) {
  if (handle.id == 0 || handle.id >= MAX_TEXTURES || !r->textures[handle.id])
    return;
  glDeleteTextures(1, &r->textures[handle.id]);
  r->textures[handle.id] = 0;
}

static void set_vec3(GLuint program, const char* name, vec3_t v) {
  glUniform3f(glGetUniformLocation(program, name), v.x, v.y, v.z);
}

void renderer_begin_frame(renderer_t* r, render_camera_t camera,
                          render_environment_t env) {
  r->environment = env;
  r->eye = camera.eye;
  r->view = mat4_look_at(camera.eye, camera.target, camera.up);
  float aspect = (float)r->width / (float)r->height;
  r->projection = mat4_perspective(camera.fov_y, aspect, camera.near_plane,
                                   camera.far_plane);
  r->view_projection = mat4_multiply(r->projection, r->view);
  r->line_count = 0;
  r->text_count = 0;
  memset(&r->stats, 0, sizeof(r->stats));
  r->stats.meshes_uploaded = r->meshes_uploaded;

  vec3_t sky = env.sky_color;
  glClearColor(powf(sky.x, 1.0f / 2.2f), powf(sky.y, 1.0f / 2.2f),
               powf(sky.z, 1.0f / 2.2f), 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  glUseProgram(r->mesh_program);
  glUniformMatrix4fv(glGetUniformLocation(r->mesh_program, "u_view_projection"),
                     1, GL_FALSE, r->view_projection.m);
  set_vec3(r->mesh_program, "u_light_direction", vec3_normalize(env.direction));
  set_vec3(r->mesh_program, "u_light_color", env.color);
  glUniform1f(glGetUniformLocation(r->mesh_program, "u_ambient"), env.ambient);
  set_vec3(r->mesh_program, "u_eye", camera.eye);
  set_vec3(r->mesh_program, "u_sky_color", env.sky_color);
  glUniform1f(glGetUniformLocation(r->mesh_program, "u_fog_start"),
              env.fog_start);
  glUniform1f(glGetUniformLocation(r->mesh_program, "u_fog_end"), env.fog_end);
  glUniform1i(glGetUniformLocation(r->mesh_program, "u_texture"), 0);
}

void renderer_draw_mesh(renderer_t* r, mesh_handle_t handle,
                        material_t material, mat4_t model) {
  if (handle.id == 0 || handle.id >= MAX_MESHES || !r->meshes[handle.id].used)
    return;
  gl_mesh_t* m = &r->meshes[handle.id];
  glUseProgram(r->mesh_program);
  glUniformMatrix4fv(glGetUniformLocation(r->mesh_program, "u_model"), 1,
                     GL_FALSE, model.m);
  set_vec3(r->mesh_program, "u_base_color", material.base_color);
  glUniform1f(glGetUniformLocation(r->mesh_program, "u_metallic"),
              material.metallic);
  glUniform1f(glGetUniformLocation(r->mesh_program, "u_roughness"),
              material.roughness);
  bool textured = material.texture.id != 0 &&
                  material.texture.id < MAX_TEXTURES &&
                  r->textures[material.texture.id] != 0;
  glUniform1i(glGetUniformLocation(r->mesh_program, "u_has_texture"),
              textured ? 1 : 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D,
                textured ? r->textures[material.texture.id] : r->white_texture);
  glBindVertexArray(m->vao);
  glDrawElements(GL_TRIANGLES, m->index_count, GL_UNSIGNED_INT, 0);
  r->stats.draw_calls++;
  r->stats.triangles += m->index_count / 3;
}

void renderer_draw_line(renderer_t* r, vec3_t a, vec3_t b, vec3_t color) {
  if (r->line_count + 2 > MAX_LINE_VERTICES) return;
  line_vertex_t va = {a, color}, vb = {b, color};
  r->line_vertices[r->line_count++] = va;
  r->line_vertices[r->line_count++] = vb;
}

void renderer_draw_box_outline(renderer_t* r, mat4_t model, vec3_t h,
                               vec3_t color) {
  vec3_t corners[8];
  for (int i = 0; i < 8; i++)
    corners[i] = mat4_transform_point(
        model,
        vec3((i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z));
  static const int edges[12][2] = {{0, 1}, {1, 3}, {3, 2}, {2, 0},
                                   {4, 5}, {5, 7}, {7, 6}, {6, 4},
                                   {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (int i = 0; i < 12; i++)
    renderer_draw_line(r, corners[edges[i][0]], corners[edges[i][1]], color);
}

static void push_text_quad(renderer_t* r, float x0, float y0, float x1,
                           float y1, vec3_t color) {
  if (r->text_count + 6 > MAX_TEXT_VERTICES) return;
  text_vertex_t a = {x0, y0, color}, b = {x1, y0, color}, c = {x1, y1, color},
                d = {x0, y1, color};
  r->text_vertices[r->text_count++] = a;
  r->text_vertices[r->text_count++] = b;
  r->text_vertices[r->text_count++] = c;
  r->text_vertices[r->text_count++] = a;
  r->text_vertices[r->text_count++] = c;
  r->text_vertices[r->text_count++] = d;
}

void renderer_draw_text(renderer_t* r, float x, float y, float scale,
                        vec3_t color, const char* text) {
  float cursor = x;
  for (const char* c = text; *c; c++) {
    if (*c == '\n') {
      cursor = x;
      y += (FONT5X7_HEIGHT + 2) * scale;
      continue;
    }
    for (int row = 0; row < FONT5X7_HEIGHT; row++)
      for (int column = 0; column < FONT5X7_WIDTH; column++)
        if (font5x7_pixel(*c, row, column))
          push_text_quad(r, cursor + column * scale, y + row * scale,
                         cursor + (column + 1) * scale, y + (row + 1) * scale,
                         color);
    cursor += (FONT5X7_WIDTH + 1) * scale;
  }
}

void renderer_end_frame(renderer_t* r) {
  if (r->line_count > 0) {
    glUseProgram(r->line_program);
    glUniformMatrix4fv(
        glGetUniformLocation(r->line_program, "u_view_projection"), 1, GL_FALSE,
        r->view_projection.m);
    glBindVertexArray(r->line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->line_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    (GLsizeiptr)(r->line_count * sizeof(line_vertex_t)),
                    r->line_vertices);
    glDisable(GL_DEPTH_TEST);
    glDrawArrays(GL_LINES, 0, (GLsizei)r->line_count);
    r->stats.draw_calls++;
  }
  if (r->text_count > 0) {
    glUseProgram(r->text_program);
    glUniform2f(glGetUniformLocation(r->text_program, "u_viewport"),
                (float)r->width, (float)r->height);
    glBindVertexArray(r->text_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->text_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    (GLsizeiptr)(r->text_count * sizeof(text_vertex_t)),
                    r->text_vertices);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)r->text_count);
    r->stats.draw_calls++;
  }
  glBindVertexArray(0);
  check_gl("renderer_end_frame");
}

mat4_t renderer_view_projection(const renderer_t* r) {
  return r->view_projection;
}

render_stats_t renderer_stats(const renderer_t* r) { return r->stats; }

bool renderer_read_pixels(renderer_t* r, uint8_t* rgba, size_t size) {
  if (size < (size_t)r->width * (size_t)r->height * 4) return false;
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, r->width, r->height, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
  check_gl("renderer_read_pixels");
  return true;
}
