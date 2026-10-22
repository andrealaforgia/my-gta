/* See gl_post.h. OpenGL 4.1 core. */
#include "gl_post.h"

#include <stdlib.h>
#include <string.h>

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#define GL_GLEXT_PROTOTYPES 1
#include <GL/gl.h>
#endif

#include "log.h"

#define BLOOM_LEVELS 3

typedef struct {
  GLuint fbo, texture;
  int width, height;
} target_t;

struct gl_post {
  int width, height, samples;
  /* The scene, multisampled. */
  GLuint scene_fbo, scene_color, scene_depth;
  /* The scene resolved, with its depth as a texture. */
  GLuint resolve_fbo, hdr_texture, depth_texture;
  target_t ao[2];
  target_t bloom[BLOOM_LEVELS][2];
  GLuint bright_program, down_program, blur_program, ssao_program,
      ao_blur_program, composite_program;
};

static const char* fullscreen_vertex =
    "#version 410 core\n"
    "out vec2 v_uv;\n"
    "void main() {\n"
    "  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);\n"
    "  v_uv = p;\n"
    "  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);\n"
    "}\n";

/* What is brighter than the screen shows, soft at the edge. */
static const char* bright_fragment =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_source;\n"
    "out vec4 out_color;\n"
    "void main() {\n"
    "  vec3 c = texture(u_source, v_uv).rgb;\n"
    "  float l = max(c.r, max(c.g, c.b));\n"
    "  float k = clamp((l - 1.1) / 1.4, 0.0, 1.0);\n"
    "  out_color = vec4(c * k, 1.0);\n"
    "}\n";

static const char* down_fragment =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_source;\n"
    "uniform vec2 u_texel;\n"
    "out vec4 out_color;\n"
    "void main() {\n"
    "  vec3 c = texture(u_source, v_uv + u_texel * vec2(-0.5, -0.5)).rgb\n"
    "         + texture(u_source, v_uv + u_texel * vec2(0.5, -0.5)).rgb\n"
    "         + texture(u_source, v_uv + u_texel * vec2(-0.5, 0.5)).rgb\n"
    "         + texture(u_source, v_uv + u_texel * vec2(0.5, 0.5)).rgb;\n"
    "  out_color = vec4(c * 0.25, 1.0);\n"
    "}\n";

static const char* blur_fragment =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_source;\n"
    "uniform vec2 u_step;\n"
    "out vec4 out_color;\n"
    "void main() {\n"
    "  vec3 c = texture(u_source, v_uv).rgb * 0.2270270270;\n"
    "  c += texture(u_source, v_uv + u_step * 1.3846153846).rgb * "
    "0.3162162162;\n"
    "  c += texture(u_source, v_uv - u_step * 1.3846153846).rgb * "
    "0.3162162162;\n"
    "  c += texture(u_source, v_uv + u_step * 3.2307692308).rgb * "
    "0.0702702703;\n"
    "  c += texture(u_source, v_uv - u_step * 3.2307692308).rgb * "
    "0.0702702703;\n"
    "  out_color = vec4(c, 1.0);\n"
    "}\n";

/* Screen-space ambient occlusion from the depth buffer alone: the view-space
 * position is rebuilt, the normal from its neighbours, and samples in the
 * hemisphere above the surface count how many lie behind something. */
static const char* ssao_fragment =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_depth;\n"
    "uniform mat4 u_projection;\n"
    "uniform mat4 u_inverse_projection;\n"
    "uniform vec2 u_texel;\n"
    "uniform float u_radius;\n"
    "out vec4 out_color;\n"
    "vec3 view_position(vec2 uv) {\n"
    "  float d = texture(u_depth, uv).r;\n"
    "  vec4 v = u_inverse_projection * vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, "
    "1.0);\n"
    "  return v.xyz / v.w;\n"
    "}\n"
    "void main() {\n"
    "  float depth = texture(u_depth, v_uv).r;\n"
    "  if (depth >= 0.99999) { out_color = vec4(1.0); return; }\n"
    "  vec3 p = view_position(v_uv);\n"
    "  vec3 pr = view_position(v_uv + vec2(u_texel.x, 0.0));\n"
    "  vec3 pl = view_position(v_uv - vec2(u_texel.x, 0.0));\n"
    "  vec3 pu = view_position(v_uv + vec2(0.0, u_texel.y));\n"
    "  vec3 pd = view_position(v_uv - vec2(0.0, u_texel.y));\n"
    "  vec3 dx = abs(pr.z - p.z) < abs(pl.z - p.z) ? pr - p : p - pl;\n"
    "  vec3 dy = abs(pu.z - p.z) < abs(pd.z - p.z) ? pu - p : p - pd;\n"
    "  vec3 n = normalize(cross(dx, dy));\n"
    "  if (dot(n, -p) < 0.0) n = -n;\n"
    "  float angle = 6.2831853 * fract(52.9829189 * fract(dot(gl_FragCoord.xy, "
    "vec2(0.06711056, 0.00583715))));\n"
    "  vec3 t = normalize(abs(n.y) < 0.9 ? cross(n, vec3(0, 1, 0)) : cross(n, "
    "vec3(1, 0, 0)));\n"
    "  vec3 b = cross(n, t);\n"
    "  float occlusion = 0.0;\n"
    "  for (int i = 0; i < 12; i++) {\n"
    "    float u1 = (float(i) + 0.5) / 12.0;\n"
    "    float phi = float(i) * 2.39996323 + angle;\n"
    "    float r = sqrt(u1);\n"
    "    vec3 h = vec3(r * cos(phi), r * sin(phi), sqrt(1.0 - u1));\n"
    "    float scale = mix(0.15, 1.0, u1 * u1);\n"
    "    vec3 sp = p + (t * h.x + b * h.y + n * h.z) * u_radius * scale;\n"
    "    vec4 clip = u_projection * vec4(sp, 1.0);\n"
    "    vec2 suv = clip.xy / clip.w * 0.5 + 0.5;\n"
    "    float scene_z = view_position(suv).z;\n"
    "    float range = smoothstep(0.0, 1.0, u_radius / max(abs(p.z - scene_z), "
    "0.001));\n"
    "    occlusion += (scene_z >= sp.z + 0.04 ? 1.0 : 0.0) * range;\n"
    "  }\n"
    "  float ao = 1.0 - occlusion / 12.0;\n"
    "  float fade = 1.0 - smoothstep(40.0, 120.0, -p.z);\n"
    "  out_color = vec4(vec3(mix(1.0, ao * ao, fade)), 1.0);\n"
    "}\n";

/* Smooths the occlusion over a few pixels, without bleeding across edges. */
static const char* ao_blur_fragment =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_ao;\n"
    "uniform sampler2D u_depth;\n"
    "uniform vec2 u_texel;\n"
    "out vec4 out_color;\n"
    "void main() {\n"
    "  float centre = texture(u_depth, v_uv).r;\n"
    "  float sum = 0.0, weight = 0.0;\n"
    "  for (int x = -2; x <= 2; x++)\n"
    "    for (int y = -2; y <= 2; y++) {\n"
    "      vec2 uv = v_uv + vec2(x, y) * u_texel;\n"
    "      float w = exp(-abs(texture(u_depth, uv).r - centre) * 40000.0);\n"
    "      sum += texture(u_ao, uv).r * w;\n"
    "      weight += w;\n"
    "    }\n"
    "  out_color = vec4(vec3(sum / max(weight, 0.0001)), 1.0);\n"
    "}\n";

static const char* composite_fragment =
    "#version 410 core\n"
    "in vec2 v_uv;\n"
    "uniform sampler2D u_hdr;\n"
    "uniform sampler2D u_ao;\n"
    "uniform sampler2D u_bloom0;\n"
    "uniform sampler2D u_bloom1;\n"
    "uniform sampler2D u_bloom2;\n"
    "uniform float u_exposure;\n"
    "uniform float u_bloom;\n"
    "uniform float u_ao_strength;\n"
    "uniform float u_vignette;\n"
    "uniform float u_saturation;\n"
    "out vec4 out_color;\n"
    "vec3 aces(vec3 x) {\n"
    "  return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14),\n"
    "               0.0, 1.0);\n"
    "}\n"
    "void main() {\n"
    "  vec3 c = texture(u_hdr, v_uv).rgb;\n"
    "  float ao = texture(u_ao, v_uv).r;\n"
    "  c *= mix(1.0, ao, u_ao_strength);\n"
    "  c += (texture(u_bloom0, v_uv).rgb * 0.5 + texture(u_bloom1, v_uv).rgb * "
    "0.35 + texture(u_bloom2, v_uv).rgb * 0.3) * u_bloom;\n"
    "  c *= u_exposure;\n"
    "  vec3 mapped = aces(c);\n"
    "  float l = dot(mapped, vec3(0.2126, 0.7152, 0.0722));\n"
    "  mapped = mix(vec3(l), mapped, u_saturation);\n"
    "  /* A touch of warmth in the lights and cool in the shade. */\n"
    "  mapped += vec3(0.012, 0.004, -0.012) * l - vec3(0.008, 0.002, -0.012) * "
    "(1.0 - l) * 0.5;\n"
    "  vec2 q = v_uv - 0.5;\n"
    "  mapped *= 1.0 - u_vignette * smoothstep(0.25, 0.85, dot(q, q) * 1.8);\n"
    "  out_color = vec4(pow(max(mapped, 0.0), vec3(1.0 / 2.2)), 1.0);\n"
    "}\n";

static GLuint compile(GLenum type, const char* source, const char* name) {
  GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, NULL);
  glCompileShader(shader);
  GLint ok = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char message[2048];
    glGetShaderInfoLog(shader, sizeof(message), NULL, message);
    LOG_ERROR("post", "shader %s failed to compile: %s", name, message);
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

static GLuint program(const char* fragment, const char* name) {
  GLuint vs = compile(GL_VERTEX_SHADER, fullscreen_vertex, name);
  GLuint fs = compile(GL_FRAGMENT_SHADER, fragment, name);
  if (!vs || !fs) return 0;
  GLuint p = glCreateProgram();
  glAttachShader(p, vs);
  glAttachShader(p, fs);
  glLinkProgram(p);
  glDeleteShader(vs);
  glDeleteShader(fs);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (!ok) {
    LOG_ERROR("post", "program %s failed to link", name);
    glDeleteProgram(p);
    return 0;
  }
  return p;
}

static bool make_target(target_t* t, int width, int height, GLenum format,
                        GLenum type) {
  t->width = width > 1 ? width : 1;
  t->height = height > 1 ? height : 1;
  glGenTextures(1, &t->texture);
  glBindTexture(GL_TEXTURE_2D, t->texture);
  glTexImage2D(GL_TEXTURE_2D, 0, (GLint)format, t->width, t->height, 0, GL_RGBA,
               type, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glGenFramebuffers(1, &t->fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, t->fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         t->texture, 0);
  return glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
}

static void free_target(target_t* t) {
  if (t->fbo) glDeleteFramebuffers(1, &t->fbo);
  if (t->texture) glDeleteTextures(1, &t->texture);
  memset(t, 0, sizeof(*t));
}

void gl_post_destroy(gl_post_t* p) {
  if (!p) return;
  if (p->scene_fbo) glDeleteFramebuffers(1, &p->scene_fbo);
  if (p->scene_color) glDeleteRenderbuffers(1, &p->scene_color);
  if (p->scene_depth) glDeleteRenderbuffers(1, &p->scene_depth);
  if (p->resolve_fbo) glDeleteFramebuffers(1, &p->resolve_fbo);
  if (p->hdr_texture) glDeleteTextures(1, &p->hdr_texture);
  if (p->depth_texture) glDeleteTextures(1, &p->depth_texture);
  for (int i = 0; i < 2; i++) free_target(&p->ao[i]);
  for (int l = 0; l < BLOOM_LEVELS; l++)
    for (int i = 0; i < 2; i++) free_target(&p->bloom[l][i]);
  GLuint programs[] = {p->bright_program,  p->down_program,
                       p->blur_program,    p->ssao_program,
                       p->ao_blur_program, p->composite_program};
  for (size_t i = 0; i < sizeof(programs) / sizeof(programs[0]); i++)
    if (programs[i]) glDeleteProgram(programs[i]);
  free(p);
}

gl_post_t* gl_post_create(int width, int height, int samples) {
  gl_post_t* p = calloc(1, sizeof(gl_post_t));
  if (!p) return NULL;
  p->width = width > 1 ? width : 1;
  p->height = height > 1 ? height : 1;
  GLint max_samples = 1;
  glGetIntegerv(GL_MAX_SAMPLES, &max_samples);
  p->samples = samples < max_samples ? samples : max_samples;
  p->bright_program = program(bright_fragment, "bright");
  p->down_program = program(down_fragment, "down");
  p->blur_program = program(blur_fragment, "blur");
  p->ssao_program = program(ssao_fragment, "ssao");
  p->ao_blur_program = program(ao_blur_fragment, "ao blur");
  p->composite_program = program(composite_fragment, "composite");
  if (!p->bright_program || !p->down_program || !p->blur_program ||
      !p->ssao_program || !p->ao_blur_program || !p->composite_program) {
    gl_post_destroy(p);
    return NULL;
  }
  /* The multisampled scene. */
  glGenRenderbuffers(1, &p->scene_color);
  glBindRenderbuffer(GL_RENDERBUFFER, p->scene_color);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, p->samples, GL_RGBA16F,
                                   p->width, p->height);
  glGenRenderbuffers(1, &p->scene_depth);
  glBindRenderbuffer(GL_RENDERBUFFER, p->scene_depth);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, p->samples,
                                   GL_DEPTH_COMPONENT32F, p->width, p->height);
  glGenFramebuffers(1, &p->scene_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, p->scene_fbo);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                            GL_RENDERBUFFER, p->scene_color);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, p->scene_depth);
  bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  /* Resolved colour and depth. */
  glGenTextures(1, &p->hdr_texture);
  glBindTexture(GL_TEXTURE_2D, p->hdr_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, p->width, p->height, 0, GL_RGBA,
               GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glGenTextures(1, &p->depth_texture);
  glBindTexture(GL_TEXTURE_2D, p->depth_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, p->width, p->height, 0,
               GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glGenFramebuffers(1, &p->resolve_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, p->resolve_fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         p->hdr_texture, 0);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D,
                         p->depth_texture, 0);
  ok =
      ok && glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  /* Occlusion at half size, and the bloom chain: half, quarter, eighth. */
  for (int i = 0; i < 2; i++)
    ok = ok && make_target(&p->ao[i], p->width / 2, p->height / 2, GL_RGBA8,
                           GL_UNSIGNED_BYTE);
  for (int l = 0; l < BLOOM_LEVELS; l++)
    for (int i = 0; i < 2; i++)
      ok = ok && make_target(&p->bloom[l][i], p->width >> (l + 1),
                             p->height >> (l + 1), GL_RGBA16F, GL_FLOAT);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  if (!ok) {
    LOG_ERROR("post", "post-processing framebuffers are incomplete");
    gl_post_destroy(p);
    return NULL;
  }
  LOG_INFO("post", "%dx multisampled HDR scene, ambient occlusion, bloom",
           p->samples);
  return p;
}

void gl_post_bind_scene(gl_post_t* p) {
  glBindFramebuffer(GL_FRAMEBUFFER, p->scene_fbo);
  glViewport(0, 0, p->width, p->height);
}

static void pass(const target_t* t, GLuint prog) {
  glBindFramebuffer(GL_FRAMEBUFFER, t->fbo);
  glViewport(0, 0, t->width, t->height);
  glUseProgram(prog);
}

static void bind(int unit, GLuint texture) {
  glActiveTexture(GL_TEXTURE0 + (GLenum)unit);
  glBindTexture(GL_TEXTURE_2D, texture);
}

static void uniform1i(GLuint prog, const char* name, int v) {
  glUniform1i(glGetUniformLocation(prog, name), v);
}
static void uniform1f(GLuint prog, const char* name, float v) {
  glUniform1f(glGetUniformLocation(prog, name), v);
}
static void uniform2f(GLuint prog, const char* name, float x, float y) {
  glUniform2f(glGetUniformLocation(prog, name), x, y);
}

void gl_post_finish(gl_post_t* p, const float* projection,
                    const float* inverse_projection, unsigned fullscreen_vao,
                    const gl_post_params_t* params) {
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glDisable(GL_BLEND);
  /* Resolve the samples: colour and depth. */
  glBindFramebuffer(GL_READ_FRAMEBUFFER, p->scene_fbo);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, p->resolve_fbo);
  glBlitFramebuffer(0, 0, p->width, p->height, 0, 0, p->width, p->height,
                    GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT, GL_NEAREST);
  glBindVertexArray(fullscreen_vao);
  /* Occlusion, then blurred. */
  if (params->ao_strength > 0) {
    pass(&p->ao[0], p->ssao_program);
    bind(0, p->depth_texture);
    uniform1i(p->ssao_program, "u_depth", 0);
    glUniformMatrix4fv(glGetUniformLocation(p->ssao_program, "u_projection"), 1,
                       GL_FALSE, projection);
    glUniformMatrix4fv(
        glGetUniformLocation(p->ssao_program, "u_inverse_projection"), 1,
        GL_FALSE, inverse_projection);
    uniform2f(p->ssao_program, "u_texel", 1.0f / (float)p->ao[0].width,
              1.0f / (float)p->ao[0].height);
    uniform1f(p->ssao_program, "u_radius", params->ao_radius);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    pass(&p->ao[1], p->ao_blur_program);
    bind(0, p->ao[0].texture);
    bind(1, p->depth_texture);
    uniform1i(p->ao_blur_program, "u_ao", 0);
    uniform1i(p->ao_blur_program, "u_depth", 1);
    uniform2f(p->ao_blur_program, "u_texel", 1.0f / (float)p->ao[1].width,
              1.0f / (float)p->ao[1].height);
    glDrawArrays(GL_TRIANGLES, 0, 3);
  }
  /* Bloom: bright parts, shrunk and blurred at three sizes. */
  if (params->bloom > 0) {
    pass(&p->bloom[0][0], p->bright_program);
    bind(0, p->hdr_texture);
    uniform1i(p->bright_program, "u_source", 0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    for (int l = 0; l < BLOOM_LEVELS; l++) {
      if (l > 0) {
        pass(&p->bloom[l][0], p->down_program);
        bind(0, p->bloom[l - 1][0].texture);
        uniform1i(p->down_program, "u_source", 0);
        uniform2f(p->down_program, "u_texel",
                  1.0f / (float)p->bloom[l - 1][0].width,
                  1.0f / (float)p->bloom[l - 1][0].height);
        glDrawArrays(GL_TRIANGLES, 0, 3);
      }
      for (int direction = 0; direction < 2; direction++) {
        pass(&p->bloom[l][1 - direction], p->blur_program);
        bind(0, p->bloom[l][direction].texture);
        uniform1i(p->blur_program, "u_source", 0);
        uniform2f(p->blur_program, "u_step",
                  direction == 0 ? 1.0f / (float)p->bloom[l][0].width : 0.0f,
                  direction == 1 ? 1.0f / (float)p->bloom[l][0].height : 0.0f);
        glDrawArrays(GL_TRIANGLES, 0, 3);
      }
    }
  }
  /* The finished image. */
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, p->width, p->height);
  glUseProgram(p->composite_program);
  bind(0, p->hdr_texture);
  bind(1, params->ao_strength > 0 ? p->ao[1].texture : p->hdr_texture);
  bind(2, p->bloom[0][0].texture);
  bind(3, p->bloom[1][0].texture);
  bind(4, p->bloom[2][0].texture);
  uniform1i(p->composite_program, "u_hdr", 0);
  uniform1i(p->composite_program, "u_ao", 1);
  uniform1i(p->composite_program, "u_bloom0", 2);
  uniform1i(p->composite_program, "u_bloom1", 3);
  uniform1i(p->composite_program, "u_bloom2", 4);
  uniform1f(p->composite_program, "u_exposure", params->exposure);
  uniform1f(p->composite_program, "u_bloom", params->bloom);
  uniform1f(p->composite_program, "u_ao_strength",
            params->ao_strength > 0 ? params->ao_strength : 0.0f);
  uniform1f(p->composite_program, "u_vignette", params->vignette);
  uniform1f(p->composite_program, "u_saturation", params->saturation);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glBindVertexArray(0);
  glActiveTexture(GL_TEXTURE0);
}
