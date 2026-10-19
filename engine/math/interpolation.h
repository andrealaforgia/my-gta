/**
 * @file interpolation.h
 * @brief Scalar helpers for smoothing and clamping.
 */
#ifndef ENGINE_MATH_INTERPOLATION_H_
#define ENGINE_MATH_INTERPOLATION_H_

float lerpf(float a, float b, float t);
float clampf(float value, float lo, float hi);
float move_towards(float value, float target, float max_step);
/* Exponential approach: the gap shrinks by e^-rate each second. Frame-rate
 * independent and never overshoots. */
float damp_towards(float value, float target, float rate, float dt);
float signf(float value);

#endif  // ENGINE_MATH_INTERPOLATION_H_
