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

#define MAX_MESHES 16384
#define MAX_TEXTURES 256
#define MAX_LINE_VERTICES (1 << 16)
#define MAX_TEXT_VERTICES (1 << 17)
#define MAX_SCREEN_IMAGES 8
#define MAX_DRAWS (1 << 15)
#define SHADOW_MAP_SIZE 2048

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

typedef struct {
  uint32_t texture; /* 0 for the white texture */
  float x, y, width, height;
  vec3_t tint;
  float alpha;
  float u0, v0, u1, v1;
} screen_image_t;

typedef struct {
  uint32_t mesh;
  material_t material;
  mat4_t model;
} draw_t;

struct renderer {
  int width, height;
  GLuint mesh_program, line_program, text_program, sky_program, sky_vao;
  gl_mesh_t meshes[MAX_MESHES];
  GLuint textures[MAX_TEXTURES];
  GLuint white_texture;
  GLuint line_vao, line_vbo;
  line_vertex_t* line_vertices;
  size_t line_count;
  line_vertex_t* tested_vertices;
  size_t tested_count;
  GLuint text_vao, text_vbo;
  GLuint image_program, image_vao, image_vbo;
  screen_image_t images[MAX_SCREEN_IMAGES];
  size_t image_count;
  text_vertex_t* text_vertices;
  size_t text_count;
  mat4_t view, projection, view_projection;
  vec3_t eye;
  vec3_t focus;
  render_environment_t environment;
  draw_t* draws;
  size_t draw_count;
  GLuint shadow_program;
  GLuint shadow_fbo;
  GLuint shadow_texture;
  mat4_t light_view_projection;
  render_stats_t stats;
  render_stats_t last_stats;
  int meshes_uploaded;
};

static const char* mesh_vertex_source =
    "#version 410 core\n"
    "layout(location = 0) in vec3 in_position;\n"
    "layout(location = 1) in vec3 in_normal;\n"
    "layout(location = 2) in vec2 in_uv;\n"
    "uniform mat4 u_model;\n"
    "uniform mat4 u_view_projection;\n"
    "uniform mat4 u_light_view_projection;\n"
    "out vec3 v_world_position;\n"
    "out vec3 v_normal;\n"
    "out vec2 v_uv;\n"
    "out vec4 v_light_position;\n"
    "void main() {\n"
    "  vec4 world = u_model * vec4(in_position, 1.0);\n"
    "  v_world_position = world.xyz;\n"
    "  v_normal = mat3(u_model) * in_normal;\n"
    "  v_uv = in_uv;\n"
    "  v_light_position = u_light_view_projection * world;\n"
    "  gl_Position = u_view_projection * world;\n"
    "}\n";

static const char* mesh_fragment_source =
    "#version 410 core\n"
    "in vec3 v_world_position;\n"
    "in vec3 v_normal;\n"
    "in vec2 v_uv;\n"
    "in vec4 v_light_position;\n"
    "uniform sampler2DShadow u_shadow_map;\n"
    "uniform int u_has_shadows;\n"
    "uniform int u_debug_shadows;\n"
    "uniform vec3 u_base_color;\n"
    "uniform float u_metallic;\n"
    "uniform float u_roughness;\n"
    "uniform float u_emissive;\n"
    "uniform float u_window_glow;\n"
    "uniform float u_wall_ao;\n"
    "uniform int u_has_texture;\n"
    "uniform sampler2D u_texture;\n"
    "uniform vec3 u_light_direction;\n"
    "uniform vec3 u_light_color;\n"
    "uniform float u_ambient;\n"
    "uniform vec3 u_eye;\n"
    "uniform vec3 u_sky_color;\n"
    "uniform vec3 u_spot_position;\n"
    "uniform vec3 u_spot_direction;\n"
    "uniform float u_spot_intensity;\n"
    "uniform float u_fog_start;\n"
    "uniform float u_fog_end;\n"
    "out vec4 out_color;\n"
    "float shadow_factor(vec3 n, vec3 l) {\n"
    "  if (u_has_shadows == 0) return 1.0;\n"
    "  vec3 p = v_light_position.xyz / v_light_position.w * 0.5 + 0.5;\n"
    "  if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0 || p.z > 1.0) "
    "return 1.0;\n"
    "  float bias = max(0.0006 * (1.0 - dot(n, l)), 0.0002);\n"
    "  float texel = 1.0 / 2048.0;\n"
    "  float lit = 0.0;\n"
    "  for (int x = -1; x <= 1; x++)\n"
    "    for (int y = -1; y <= 1; y++)\n"
    "      lit += texture(u_shadow_map, vec3(p.xy + vec2(x, y) * texel, p.z - "
    "bias));\n"
    "  return lit / 9.0;\n"
    "}\n"
    "void main() {\n"
    "  vec3 albedo = u_base_color;\n"
    "  float pane = 1.0;\n"
    "  if (u_has_texture == 1) {\n"
    "    vec4 texel = texture(u_texture, v_uv);\n"
    "    albedo *= texel.rgb;\n"
    "    pane = texel.a;\n"
    "  }\n"
    "  vec3 n = normalize(v_normal);\n"
    "  bool shiny = u_roughness < 0.25 && u_metallic > 0.3;\n"
    "  if (shiny && u_roughness < 0.15 && n.y > 0.9) {\n"
    "    n = normalize(n + vec3(0.05 * sin(v_world_position.x * 1.9 + "
    "v_world_position.z * 0.7), 0.0,\n"
    "                           0.05 * sin(v_world_position.z * 1.4 - "
    "v_world_position.x * 0.9)));\n"
    "  }\n"
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
    "  float shadow = shadow_factor(n, l);\n"
    "  vec3 color = hemisphere * u_ambient\n"
    "             + (diffuse * n_dot_l + specular_tint * spec * n_dot_l) * "
    "u_light_color * shadow;\n"
    "  if (shiny) {\n"
    "    float fresnel = pow(1.0 - max(dot(n, v), 0.0), 4.0);\n"
    "    color = mix(color, u_sky_color * (0.35 + 0.65 * u_ambient / 0.3), "
    "0.06 + 0.5 * fresnel);\n"
    "  }\n"
    "  if (u_wall_ao > 0.0) {\n"
    "    float up = max(v_uv.y * 3.2, 0.0);\n"
    "    color *= mix(1.0 - 0.42 * u_wall_ao, 1.0, smoothstep(0.0, 1.8, up));\n"
    "  }\n"
    "  color += albedo * u_emissive;\n"
    "  if (u_window_glow > 0.0 && pane < 0.5) {\n"
    "    vec3 cell = floor(vec3(v_world_position.x + v_world_position.z, "
    "v_world_position.y, 0.0) * vec3(0.33, 0.3, 1.0));\n"
    "    float h = fract(sin(dot(cell.xy, vec2(12.9898, 78.233))) * "
    "43758.5453);\n"
    "    float lit = step(0.52, h) * u_window_glow;\n"
    "    color = mix(color, vec3(1.0, 0.78, 0.42) * (0.7 + 0.5 * h), lit);\n"
    "  }\n"
    "  if (u_spot_intensity > 0.0) {\n"
    "    vec3 to_point = v_world_position - u_spot_position;\n"
    "    float d = length(to_point);\n"
    "    vec3 ray = to_point / max(d, 0.001);\n"
    "    float cone = smoothstep(0.55, 0.93, dot(ray, "
    "normalize(u_spot_direction)));\n"
    "    float facing = max(dot(n, -ray), 0.0);\n"
    "    float falloff = 1.0 / (1.0 + 0.012 * d * d);\n"
    "    color += albedo * vec3(1.0, 0.94, 0.78) * u_spot_intensity * cone * "
    "facing * falloff;\n"
    "  }\n"
    "  float distance = length(u_eye - v_world_position);\n"
    "  float fog = clamp((distance - u_fog_start) / (u_fog_end - u_fog_start), "
    "0.0, 1.0);\n"
    "  color = mix(color, u_sky_color, fog);\n"
    "  if (u_debug_shadows == 1) color = vec3(shadow);\n"
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

static const char* image_vertex_source =
    "#version 410 core\n"
    "layout(location = 0) in vec2 in_position;\n"
    "layout(location = 1) in vec2 in_uv;\n"
    "uniform vec2 u_viewport;\n"
    "out vec2 v_uv;\n"
    "void main() {\n"
    "  v_uv = in_uv;\n"
    "  vec2 ndc = vec2(in_position.x / u_viewport.x * 2.0 - 1.0, 1.0 - "
    "in_position.y / u_viewport.y * 2.0);\n"
    "  gl_Position = vec4(ndc, 0.0, 1.0);\n"
    "}\n";

static const char* image_fragment_source =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_texture;\n"
    "uniform vec3 u_tint;\n"
    "uniform float u_alpha;\n"
    "out vec4 out_color;\n"
    "void main() { out_color = vec4(texture(u_texture, v_uv).rgb * u_tint, "
    "u_alpha); }\n";

static const char* sky_vertex_source =
    "#version 410 core\n"
    "out vec2 v_ndc;\n"
    "void main() {\n"
    "  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);\n"
    "  v_ndc = p * 2.0 - 1.0;\n"
    "  gl_Position = vec4(v_ndc, 1.0, 1.0);\n"
    "}\n";

static const char* sky_fragment_source =
    "#version 410 core\n"
    "in vec2 v_ndc;\n"
    "uniform mat4 u_inverse_view_projection;\n"
    "uniform vec3 u_eye;\n"
    "uniform vec3 u_horizon;\n"
    "uniform vec3 u_zenith;\n"
    "uniform vec3 u_sun_direction;\n"
    "uniform vec3 u_sun_color;\n"
    "uniform float u_stars;\n"
    "uniform float u_sun_disc;\n"
    "uniform float u_moon_disc;\n"
    "uniform float u_cloud_cover;\n"
    "uniform float u_cloud_time;\n"
    "uniform float u_cloud_light;\n"
    "out vec4 out_color;\n"
    "float hash2(vec2 p) {\n"
    "  p = fract(p * vec2(0.1031, 0.1030));\n"
    "  p += dot(p, p.yx + 33.33);\n"
    "  return fract((p.x + p.y) * p.x);\n"
    "}\n"
    "float value_noise(vec2 p) {\n"
    "  vec2 i = floor(p), f = fract(p);\n"
    "  f = f * f * (3.0 - 2.0 * f);\n"
    "  return mix(mix(hash2(i), hash2(i + vec2(1, 0)), f.x),\n"
    "             mix(hash2(i + vec2(0, 1)), hash2(i + vec2(1, 1)), f.x), "
    "f.y);\n"
    "}\n"
    "float fbm(vec2 p) {\n"
    "  float v = 0.0, a = 0.5;\n"
    "  for (int k = 0; k < 5; k++) { v += a * value_noise(p); p = p * 2.03 + "
    "11.7; a *= 0.5; }\n"
    "  return v;\n"
    "}\n"
    "float hash(vec3 p) {\n"
    "  p = fract(p * 0.3183099 + vec3(0.71, 0.113, 0.419));\n"
    "  p *= 17.0;\n"
    "  return fract(p.x * p.y * p.z * (p.x + p.y + p.z));\n"
    "}\n"
    "void main() {\n"
    "  vec4 far_point = u_inverse_view_projection * vec4(v_ndc, 1.0, 1.0);\n"
    "  vec3 dir = normalize(far_point.xyz / far_point.w - u_eye);\n"
    "  float up = max(dir.y, 0.0);\n"
    "  vec3 color = mix(u_horizon, u_zenith, pow(up, 0.55));\n"
    "  vec3 sun = normalize(u_sun_direction);\n"
    "  float along = max(dot(dir, sun), 0.0);\n"
    "  color += u_sun_color * (pow(along, 6.0) * 0.12 + pow(along, 64.0) * "
    "0.25) * u_sun_disc;\n"
    "  color += u_sun_color * smoothstep(0.9995, 0.9999, along) * 6.0 * "
    "u_sun_disc;\n"
    "  if (u_moon_disc > 0.0) {\n"
    "    float m = dot(dir, sun);\n"
    "    float edge = smoothstep(0.99978, 0.99984, m);\n"
    "    vec2 face = (dir - sun * m).xz * 700.0;\n"
    "    float maria = 0.55 + 0.45 * smoothstep(0.35, 0.7, fbm(face * 0.35));\n"
    "    vec3 moon = vec3(0.95, 0.93, 0.85) * maria * 0.95;\n"
    "    color += moon * edge * u_moon_disc;\n"
    "    color += vec3(0.5, 0.58, 0.8) * pow(max(m, 0.0), 220.0) * 0.18 * "
    "u_moon_disc;\n"
    "  }\n"
    "  if (dir.y > 0.0 && u_stars > 0.0) {\n"
    "    vec3 scaled = dir * 260.0;\n"
    "    vec3 cell = floor(scaled);\n"
    "    float h = hash(cell);\n"
    "    vec3 centre = cell + 0.25 + 0.5 * vec3(fract(h * 7.0), fract(h * "
    "13.0), fract(h * 23.0));\n"
    "    float d = length(scaled - centre);\n"
    "    float star = step(0.992, h) * smoothstep(0.55, 0.0, d) * "
    "smoothstep(0.0, 0.35, dir.y);\n"
    "    color += vec3(0.9, 0.92, 1.0) * star * u_stars * (0.4 + h);\n"
    "  }\n"
    "  if (dir.y > 0.0) {\n"
    "    vec2 sky = dir.xz / (dir.y + 0.28) * 1.1 + vec2(u_cloud_time * 0.012, "
    "u_cloud_time * 0.005);\n"
    "    float n = fbm(sky);\n"
    "    float lo = 0.74 - u_cloud_cover * 0.44;\n"
    "    float density = smoothstep(lo, lo + 0.2, n);\n"
    "    density *= smoothstep(0.0, 0.16, dir.y);\n"
    "    float thick = clamp((n - 0.45) * 2.0, 0.0, 1.0);\n"
    "    vec3 cloud = mix(vec3(1.0) * u_cloud_light, vec3(0.55, 0.58, 0.64) * "
    "u_cloud_light, thick * 0.6);\n"
    "    cloud = mix(cloud, u_horizon, 0.2);\n"
    "    color = mix(color, cloud, density * (0.55 + 0.4 * u_cloud_cover));\n"
    "  }\n"
    "  if (dir.y < 0.0) color = mix(u_horizon, u_horizon * 0.8, "
    "min(-dir.y * 4.0, 1.0));\n"
    "  out_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);\n"
    "}\n";

static const char* shadow_vertex_source =
    "#version 410 core\n"
    "layout(location = 0) in vec3 in_position;\n"
    "uniform mat4 u_model;\n"
    "uniform mat4 u_light_view_projection;\n"
    "void main() { gl_Position = u_light_view_projection * u_model * "
    "vec4(in_position, 1.0); }\n";

static const char* shadow_fragment_source =
    "#version 410 core\n"
    "void main() {}\n";

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
  r->shadow_program =
      link_program(shadow_vertex_source, shadow_fragment_source, "shadow");
  r->line_vertices = malloc(MAX_LINE_VERTICES * sizeof(line_vertex_t));
  r->sky_program = link_program(sky_vertex_source, sky_fragment_source, "sky");
  glGenVertexArrays(1, &r->sky_vao);
  r->image_program =
      link_program(image_vertex_source, image_fragment_source, "image");
  r->tested_vertices = malloc(MAX_LINE_VERTICES * sizeof(line_vertex_t));
  r->text_vertices = malloc(MAX_TEXT_VERTICES * sizeof(text_vertex_t));
  r->draws = malloc(MAX_DRAWS * sizeof(draw_t));
  if (!r->mesh_program || !r->line_program || !r->text_program ||
      !r->image_program || !r->sky_program || !r->shadow_program ||
      !r->line_vertices || !r->text_vertices || !r->draws) {
    renderer_destroy(r);
    return NULL;
  }
  glGenTextures(1, &r->shadow_texture);
  glBindTexture(GL_TEXTURE_2D, r->shadow_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, SHADOW_MAP_SIZE,
               SHADOW_MAP_SIZE, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                  GL_COMPARE_REF_TO_TEXTURE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glGenFramebuffers(1, &r->shadow_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, r->shadow_fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                         r->shadow_texture, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    LOG_ERROR("renderer", "shadow framebuffer is incomplete; shadows disabled");
    glDeleteFramebuffers(1, &r->shadow_fbo);
    r->shadow_fbo = 0;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
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

  glGenVertexArrays(1, &r->image_vao);
  glGenBuffers(1, &r->image_vbo);
  glBindVertexArray(r->image_vao);
  glBindBuffer(GL_ARRAY_BUFFER, r->image_vbo);
  glBufferData(GL_ARRAY_BUFFER, 6 * 4 * sizeof(float), NULL, GL_STREAM_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float),
                        (void*)(2 * sizeof(float)));
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
  LOG_INFO("renderer", "OpenGL %s on %s, shadow map %s",
           (const char*)glGetString(GL_VERSION),
           (const char*)glGetString(GL_RENDERER),
           r->shadow_fbo ? "ready" : "unavailable");
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
  if (r->shadow_texture) glDeleteTextures(1, &r->shadow_texture);
  if (r->shadow_fbo) glDeleteFramebuffers(1, &r->shadow_fbo);
  if (r->shadow_program) glDeleteProgram(r->shadow_program);
  free(r->draws);
  if (r->line_vbo) glDeleteBuffers(1, &r->line_vbo);
  if (r->line_vao) glDeleteVertexArrays(1, &r->line_vao);
  if (r->text_vbo) glDeleteBuffers(1, &r->text_vbo);
  if (r->text_vao) glDeleteVertexArrays(1, &r->text_vao);
  if (r->mesh_program) glDeleteProgram(r->mesh_program);
  if (r->line_program) glDeleteProgram(r->line_program);
  if (r->text_program) glDeleteProgram(r->text_program);
  free(r->line_vertices);
  free(r->tested_vertices);
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
  r->focus = camera.target;
  r->view = mat4_look_at(camera.eye, camera.target, camera.up);
  float aspect = (float)r->width / (float)r->height;
  r->projection = mat4_perspective(camera.fov_y, aspect, camera.near_plane,
                                   camera.far_plane);
  r->view_projection = mat4_multiply(r->projection, r->view);
  r->line_count = 0;
  r->tested_count = 0;
  r->text_count = 0;
  r->image_count = 0;
  r->draw_count = 0;
  memset(&r->stats, 0, sizeof(r->stats));
  r->stats.meshes_uploaded = r->meshes_uploaded;
}

static bool shadows_enabled(const renderer_t* r) {
  return r->shadow_fbo != 0 && r->environment.shadow_radius > 0;
}

/* An orthographic light box around the camera's focus, its centre snapped
 * to shadow-map texels so the shadows do not shimmer as the car moves. */
static mat4_t light_matrix(const renderer_t* r) {
  float radius = r->environment.shadow_radius;
  vec3_t direction = vec3_normalize(r->environment.direction);
  vec3_t up = fabsf(direction.y) > 0.99f ? vec3(0, 0, -1) : vec3(0, 1, 0);
  mat4_t view = mat4_look_at(
      vec3_add_scaled(r->focus, direction, radius * 2.0f), r->focus, up);
  vec3_t centre = mat4_transform_point(view, r->focus);
  float texel = radius * 2.0f / (float)SHADOW_MAP_SIZE;
  float snap_x = floorf(centre.x / texel) * texel - centre.x;
  float snap_y = floorf(centre.y / texel) * texel - centre.y;
  mat4_t projection =
      mat4_orthographic(-radius + snap_x, radius + snap_x, -radius + snap_y,
                        radius + snap_y, 0.1f, radius * 4.0f);
  return mat4_multiply(projection, view);
}

static void draw_list(renderer_t* r, GLuint program, bool bind_materials) {
  for (size_t i = 0; i < r->draw_count; i++) {
    const draw_t* draw = &r->draws[i];
    gl_mesh_t* m = &r->meshes[draw->mesh];
    glUniformMatrix4fv(glGetUniformLocation(program, "u_model"), 1, GL_FALSE,
                       draw->model.m);
    if (bind_materials) {
      const material_t* material = &draw->material;
      set_vec3(program, "u_base_color", material->base_color);
      glUniform1f(glGetUniformLocation(program, "u_metallic"),
                  material->metallic);
      glUniform1f(glGetUniformLocation(program, "u_roughness"),
                  material->roughness);
      glUniform1f(glGetUniformLocation(program, "u_emissive"),
                  material->emissive);
      glUniform1f(glGetUniformLocation(program, "u_window_glow"),
                  material->window_glow);
      glUniform1f(glGetUniformLocation(program, "u_wall_ao"),
                  material->wall_ao);
      bool textured = material->texture.id != 0 &&
                      material->texture.id < MAX_TEXTURES &&
                      r->textures[material->texture.id] != 0;
      glUniform1i(glGetUniformLocation(program, "u_has_texture"),
                  textured ? 1 : 0);
      glActiveTexture(GL_TEXTURE0);
      glBindTexture(GL_TEXTURE_2D, textured ? r->textures[material->texture.id]
                                            : r->white_texture);
    }
    glBindVertexArray(m->vao);
    glDrawElements(GL_TRIANGLES, m->index_count, GL_UNSIGNED_INT, 0);
    r->stats.draw_calls++;
    if (bind_materials) r->stats.triangles += m->index_count / 3;
  }
}

static void shadow_pass(renderer_t* r) {
  r->light_view_projection = light_matrix(r);
  glBindFramebuffer(GL_FRAMEBUFFER, r->shadow_fbo);
  glViewport(0, 0, SHADOW_MAP_SIZE, SHADOW_MAP_SIZE);
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_FRONT); /* back faces in the map: less acne on lit faces */
  glUseProgram(r->shadow_program);
  glUniformMatrix4fv(
      glGetUniformLocation(r->shadow_program, "u_light_view_projection"), 1,
      GL_FALSE, r->light_view_projection.m);
  draw_list(r, r->shadow_program, false);
  glCullFace(GL_BACK);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, r->width, r->height);
}

static void draw_sky(renderer_t* r) {
  const render_environment_t* env = &r->environment;
  glUseProgram(r->sky_program);
  mat4_t inverse = mat4_inverse(r->view_projection);
  glUniformMatrix4fv(
      glGetUniformLocation(r->sky_program, "u_inverse_view_projection"), 1,
      GL_FALSE, inverse.m);
  set_vec3(r->sky_program, "u_eye", r->eye);
  set_vec3(r->sky_program, "u_horizon", env->sky_color);
  set_vec3(r->sky_program, "u_zenith", env->zenith_color);
  set_vec3(r->sky_program, "u_sun_direction", env->direction);
  set_vec3(r->sky_program, "u_sun_color", env->color);
  glUniform1f(glGetUniformLocation(r->sky_program, "u_stars"), env->star_level);
  glUniform1f(glGetUniformLocation(r->sky_program, "u_sun_disc"),
              env->sun_disc);
  glUniform1f(glGetUniformLocation(r->sky_program, "u_moon_disc"),
              env->moon_disc);
  glUniform1f(glGetUniformLocation(r->sky_program, "u_cloud_cover"),
              env->cloud_cover);
  glUniform1f(glGetUniformLocation(r->sky_program, "u_cloud_time"),
              env->cloud_time);
  glUniform1f(glGetUniformLocation(r->sky_program, "u_cloud_light"),
              fminf(vec3_length(env->color) * 0.5f + 0.03f, 1.0f));
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glBindVertexArray(r->sky_vao);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);
}

static void main_pass(renderer_t* r) {
  const render_environment_t* env = &r->environment;
  vec3_t sky = env->sky_color;
  glClearColor(powf(sky.x, 1.0f / 2.2f), powf(sky.y, 1.0f / 2.2f),
               powf(sky.z, 1.0f / 2.2f), 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  draw_sky(r);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  GLuint program = r->mesh_program;
  glUseProgram(program);
  glUniformMatrix4fv(glGetUniformLocation(program, "u_view_projection"), 1,
                     GL_FALSE, r->view_projection.m);
  glUniformMatrix4fv(glGetUniformLocation(program, "u_light_view_projection"),
                     1, GL_FALSE, r->light_view_projection.m);
  set_vec3(program, "u_light_direction", vec3_normalize(env->direction));
  set_vec3(program, "u_light_color", env->color);
  glUniform1f(glGetUniformLocation(program, "u_ambient"), env->ambient);
  set_vec3(program, "u_eye", r->eye);
  set_vec3(program, "u_sky_color", env->sky_color);
  set_vec3(program, "u_spot_position", env->spot_position);
  set_vec3(program, "u_spot_direction", env->spot_direction);
  glUniform1f(glGetUniformLocation(program, "u_spot_intensity"),
              env->spot_intensity);
  glUniform1f(glGetUniformLocation(program, "u_fog_start"), env->fog_start);
  glUniform1f(glGetUniformLocation(program, "u_fog_end"), env->fog_end);
  glUniform1i(glGetUniformLocation(program, "u_texture"), 0);
  glUniform1i(glGetUniformLocation(program, "u_shadow_map"), 1);
  glUniform1i(glGetUniformLocation(program, "u_has_shadows"),
              shadows_enabled(r) ? 1 : 0);
  glUniform1i(glGetUniformLocation(program, "u_debug_shadows"),
              getenv("GTA_DEBUG_SHADOWS") ? 1 : 0);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, r->shadow_texture);
  draw_list(r, program, true);
}

void renderer_draw_mesh(renderer_t* r, mesh_handle_t handle,
                        material_t material, mat4_t model) {
  if (handle.id == 0 || handle.id >= MAX_MESHES || !r->meshes[handle.id].used)
    return;
  if (r->draw_count >= MAX_DRAWS) return;
  draw_t* draw = &r->draws[r->draw_count++];
  draw->mesh = handle.id;
  draw->material = material;
  draw->model = model;
}

void renderer_draw_line(renderer_t* r, vec3_t a, vec3_t b, vec3_t color) {
  if (r->line_count + 2 > MAX_LINE_VERTICES) return;
  line_vertex_t va = {a, color}, vb = {b, color};
  r->line_vertices[r->line_count++] = va;
  r->line_vertices[r->line_count++] = vb;
}

void renderer_draw_line_tested(renderer_t* r, vec3_t a, vec3_t b, vec3_t c) {
  if (!r->tested_vertices || r->tested_count + 2 > MAX_LINE_VERTICES) return;
  line_vertex_t va = {a, c}, vb = {b, c};
  r->tested_vertices[r->tested_count++] = va;
  r->tested_vertices[r->tested_count++] = vb;
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

void renderer_draw_screen_image(renderer_t* r, texture_handle_t texture,
                                float x, float y, float width, float height,
                                vec3_t tint, float alpha) {
  if (r->image_count >= MAX_SCREEN_IMAGES) return;
  renderer_draw_screen_image_uv(r, texture, x, y, width, height, 0, 0, 1, 1,
                                tint, alpha);
}

void renderer_draw_screen_image_uv(renderer_t* r, texture_handle_t texture,
                                   float x, float y, float width, float height,
                                   float u0, float v0, float u1, float v1,
                                   vec3_t tint, float alpha) {
  if (r->image_count >= MAX_SCREEN_IMAGES) return;
  screen_image_t image = {texture.id, x,  y,  width, height, tint,
                          alpha,      u0, v0, u1,    v1};
  r->images[r->image_count++] = image;
}

void renderer_draw_screen_line(renderer_t* r, float x0, float y0, float x1,
                               float y1, float thickness, vec3_t color) {
  float dx = x1 - x0, dy = y1 - y0;
  float length = sqrtf(dx * dx + dy * dy);
  if (length < 1e-4f || r->text_count + 6 > MAX_TEXT_VERTICES) return;
  float nx = -dy / length * thickness * 0.5f,
        ny = dx / length * thickness * 0.5f;
  text_vertex_t a = {x0 + nx, y0 + ny, color}, b = {x1 + nx, y1 + ny, color},
                c = {x1 - nx, y1 - ny, color}, d = {x0 - nx, y0 - ny, color};
  r->text_vertices[r->text_count++] = a;
  r->text_vertices[r->text_count++] = b;
  r->text_vertices[r->text_count++] = c;
  r->text_vertices[r->text_count++] = a;
  r->text_vertices[r->text_count++] = c;
  r->text_vertices[r->text_count++] = d;
}

static void draw_screen_images(renderer_t* r) {
  if (r->image_count == 0) return;
  glUseProgram(r->image_program);
  glUniform2f(glGetUniformLocation(r->image_program, "u_viewport"),
              (float)r->width, (float)r->height);
  glUniform1i(glGetUniformLocation(r->image_program, "u_texture"), 0);
  glBindVertexArray(r->image_vao);
  glBindBuffer(GL_ARRAY_BUFFER, r->image_vbo);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  for (size_t i = 0; i < r->image_count; i++) {
    const screen_image_t* im = &r->images[i];
    float x0 = im->x, y0 = im->y, x1 = im->x + im->width,
          y1 = im->y + im->height;
    float quad[24] = {x0, y0, im->u0, im->v0, x1, y0, im->u1, im->v0,
                      x1, y1, im->u1, im->v1, x0, y0, im->u0, im->v0,
                      x1, y1, im->u1, im->v1, x0, y1, im->u0, im->v1};
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quad), quad);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, im->texture && im->texture < MAX_TEXTURES &&
                                         r->textures[im->texture]
                                     ? r->textures[im->texture]
                                     : r->white_texture);
    glUniform3f(glGetUniformLocation(r->image_program, "u_tint"), im->tint.x,
                im->tint.y, im->tint.z);
    glUniform1f(glGetUniformLocation(r->image_program, "u_alpha"), im->alpha);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    r->stats.draw_calls++;
  }
  glDisable(GL_BLEND);
}

void renderer_end_frame(renderer_t* r) {
  if (shadows_enabled(r))
    shadow_pass(r);
  else
    r->light_view_projection = mat4_identity();
  main_pass(r);
  if (r->tested_count > 0) {
    glUseProgram(r->line_program);
    glUniformMatrix4fv(
        glGetUniformLocation(r->line_program, "u_view_projection"), 1, GL_FALSE,
        r->view_projection.m);
    glBindVertexArray(r->line_vao);
    glBindBuffer(GL_ARRAY_BUFFER, r->line_vbo);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    (GLsizeiptr)(r->tested_count * sizeof(line_vertex_t)),
                    r->tested_vertices);
    glEnable(GL_DEPTH_TEST);
    glDrawArrays(GL_LINES, 0, (GLsizei)r->tested_count);
    r->stats.draw_calls++;
  }
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
  draw_screen_images(r);
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
  r->last_stats = r->stats;
  check_gl("renderer_end_frame");
}

mat4_t renderer_view_projection(const renderer_t* r) {
  return r->view_projection;
}

render_stats_t renderer_stats(const renderer_t* r) { return r->last_stats; }

int renderer_read_shadow_map(renderer_t* r, float* depths, size_t count) {
  if (!r->shadow_fbo || count < (size_t)SHADOW_MAP_SIZE * SHADOW_MAP_SIZE)
    return 0;
  glBindTexture(GL_TEXTURE_2D, r->shadow_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, depths);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                  GL_COMPARE_REF_TO_TEXTURE);
  return SHADOW_MAP_SIZE;
}

float renderer_shadow_coverage(renderer_t* r) {
  if (!r->shadow_fbo) return 0;
  size_t count = (size_t)SHADOW_MAP_SIZE * SHADOW_MAP_SIZE;
  float* depths = malloc(count * sizeof(float));
  if (!depths) return 0;
  glBindTexture(GL_TEXTURE_2D, r->shadow_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, GL_FLOAT, depths);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE,
                  GL_COMPARE_REF_TO_TEXTURE);
  size_t written = 0;
  for (size_t i = 0; i < count; i++)
    if (depths[i] < 0.9999f) written++;
  free(depths);
  check_gl("renderer_shadow_coverage");
  return (float)written / (float)count;
}

bool renderer_read_pixels(renderer_t* r, uint8_t* rgba, size_t size) {
  if (size < (size_t)r->width * (size_t)r->height * 4) return false;
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, r->width, r->height, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
  check_gl("renderer_read_pixels");
  return true;
}
