# ADR-002: Rendering API

Status: accepted, 2026-10-20

## Decision

OpenGL 4.1 core profile through `SDL_GL_CreateContext`, behind a C renderer
interface (`engine/renderer/renderer.h`) that exposes meshes, materials,
cameras, lights and debug lines and nothing GL-specific.

## Why

The spec preferred Metal. Metal needs Objective-C; the owner's constant is C.
OpenGL 4.1 is deprecated on macOS but present in the macOS 26 SDK and works on
Apple silicon (spike on 2026-10-20: `GL_VERSION=4.1 Metal - 90.5`,
`GL_RENDERER=Apple M4 Pro`, GLSL 4.10 compiles). It is pure C, SDL2 creates
the context, and it is the same API on Linux.

## Validation (done in the spike)

Window, context, shader compile, clear, swap, `glReadPixels` all work. The
`dummy` SDL video driver cannot make a GL context, so renderer tests open a
hidden window and run only where a GPU is present; everything else is tested
without a window.

## Fallback

If Apple removes OpenGL, the renderer interface is small enough to back with
Metal through a single Objective-C file. Nothing above `renderer.h` would
change.
