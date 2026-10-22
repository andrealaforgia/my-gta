/**
 * @file gl_post.h
 * @brief The OpenGL post-processing chain behind the renderer: the scene is
 * drawn into a multisampled floating-point target, resolved, darkened where
 * surfaces meet (screen-space ambient occlusion), blooms where it is brighter
 * than the screen can show, and is mapped to the display with a filmic curve.
 */
#ifndef ENGINE_RENDERER_GL_GL_POST_H_
#define ENGINE_RENDERER_GL_GL_POST_H_

#include <stdbool.h>

typedef struct gl_post gl_post_t;

typedef struct {
  float exposure;    /* multiplies the scene before the tone curve */
  float bloom;       /* 0 for none, about 0.3 is soft */
  float ao_strength; /* 0 for none */
  float ao_radius;   /* metres */
  float vignette;    /* 0 for none */
  float saturation;  /* 1 leaves colours alone */
} gl_post_params_t;

/* Returns NULL if the framebuffers cannot be made, in which case the caller
 * draws straight to the window. */
gl_post_t* gl_post_create(int width, int height, int samples);
void gl_post_destroy(gl_post_t* post);
/* Binds the multisampled target for the scene; sets the viewport. */
void gl_post_bind_scene(gl_post_t* post);
/* Resolves the scene and draws the finished image into the default
 * framebuffer. `projection` and `inverse_projection` are column-major. */
void gl_post_finish(gl_post_t* post, const float* projection,
                    const float* inverse_projection, unsigned fullscreen_vao,
                    const gl_post_params_t* params);

#endif  // ENGINE_RENDERER_GL_GL_POST_H_
