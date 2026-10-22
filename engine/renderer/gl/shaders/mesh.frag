#version 410 core
in vec3 v_world_position;
in vec3 v_normal;
in vec2 v_uv;
in vec4 v_light_position;
uniform sampler2DShadow u_shadow_map;
uniform int u_has_shadows;
uniform int u_debug_shadows;
uniform vec3 u_base_color;
uniform float u_metallic;
uniform float u_roughness;
uniform float u_emissive;
uniform float u_window_glow;
uniform float u_wall_ao;
uniform int u_has_texture;
uniform sampler2D u_texture;
uniform vec3 u_light_direction;
uniform vec3 u_light_color;
uniform float u_ambient;
uniform vec3 u_eye;
uniform vec3 u_sky_color;
uniform vec3 u_zenith_color;
uniform float u_time;
uniform vec3 u_spot_position;
uniform vec3 u_spot_direction;
uniform float u_spot_intensity;
uniform float u_fog_start;
uniform float u_fog_end;
uniform int u_srgb_out;
out vec4 out_color;

float hash13(vec3 p) {
  p = fract(p * 0.1031);
  p += dot(p, p.zyx + 31.32);
  return fract((p.x + p.y) * p.z);
}
float vnoise(vec3 p) {
  vec3 i = floor(p), f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  return mix(mix(mix(hash13(i), hash13(i + vec3(1, 0, 0)), f.x),
                 mix(hash13(i + vec3(0, 1, 0)), hash13(i + vec3(1, 1, 0)), f.x), f.y),
             mix(mix(hash13(i + vec3(0, 0, 1)), hash13(i + vec3(1, 0, 1)), f.x),
                 mix(hash13(i + vec3(0, 1, 1)), hash13(i + vec3(1, 1, 1)), f.x), f.y),
             f.z);
}

/* Variation in the surface itself, so that a wall or a pavement is not one
 * flat colour: broad patches, a mid grain and a fine speckle, the finer
 * ones fading out with distance so they never shimmer. */
float surface_detail(vec3 wp, float distance, float up) {
  float v = (vnoise(wp * 0.45) - 0.5) * 0.16;
  /* Open ground wears in broad patches: damp, swept, stained. */
  if (up > 0.8) v += (vnoise(wp * vec3(0.11, 0.0, 0.11) + 5.3) - 0.5) * 0.24 +
                     (vnoise(wp * vec3(0.04, 0.0, 0.04) + 1.7) - 0.5) * 0.2;
  if (distance < 90.0) {
    float mid_fade = 1.0 - smoothstep(25.0, 90.0, distance);
    v += (vnoise(wp * 2.3 + 7.1) - 0.5) * 0.12 * mid_fade;
    if (distance < 30.0) {
      float fine_fade = 1.0 - smoothstep(6.0, 30.0, distance);
      v += (vnoise(wp * 9.0 + 3.7) - 0.5) * 0.14 * fine_fade;
    }
  }
  return 1.0 + v;
}

float shadow_factor(vec3 n, vec3 l) {
  if (u_has_shadows == 0) return 1.0;
  vec3 p = v_light_position.xyz / v_light_position.w * 0.5 + 0.5;
  if (p.x < 0.0 || p.x > 1.0 || p.y < 0.0 || p.y > 1.0 || p.z > 1.0) return 1.0;
  float bias = max(0.0007 * (1.0 - dot(n, l)), 0.00025);
  float texel = 1.0 / 2048.0;
  /* A different turn of the sample pattern at each pixel turns the banding
   * into fine grain. */
  float angle = 6.2831853 * fract(52.9829189 * fract(dot(gl_FragCoord.xy, vec2(0.06711056, 0.00583715))));
  mat2 rotation = mat2(cos(angle), sin(angle), -sin(angle), cos(angle));
  float lit = 0.0;
  for (int i = 0; i < 9; i++) {
    float r = sqrt((float(i) + 0.5) / 9.0);
    float a = float(i) * 2.39996323;
    vec2 offset = rotation * vec2(cos(a), sin(a)) * r * 3.0 * texel;
    lit += texture(u_shadow_map, vec3(p.xy + offset, p.z - bias));
  }
  return lit / 9.0;
}

/* Relief from the texture itself: the light and dark of a surface (the mortar
 * between bricks, cracks in asphalt, plaster grain) is taken as height and
 * tilts the normal, so that low sun picks the texture out. */
vec3 bump_normal(vec3 n, vec3 p, float height, float strength) {
  vec3 dpdx = dFdx(p), dpdy = dFdy(p);
  float dhx = dFdx(height), dhy = dFdy(height);
  vec3 r1 = cross(dpdy, n), r2 = cross(n, dpdx);
  float det = dot(dpdx, r1);
  vec3 grad = sign(det) * (dhx * r1 + dhy * r2);
  return normalize(abs(det) * n - strength * grad);
}

vec3 sky_reflection(vec3 r) {
  vec3 sky = mix(u_sky_color, u_zenith_color, smoothstep(0.0, 0.7, r.y));
  vec3 ground = u_sky_color * 0.55 + vec3(0.02);
  return mix(ground, sky, smoothstep(-0.15, 0.1, r.y));
}

void main() {
  vec3 albedo = u_base_color;
  float pane = 1.0;
  float texture_height = 0.0;
  if (u_has_texture == 1) {
    vec4 texel = texture(u_texture, v_uv);
    albedo *= texel.rgb;
    pane = texel.a;
    texture_height = dot(texel.rgb, vec3(0.3, 0.59, 0.11));
  }
  float eye_distance = length(u_eye - v_world_position);
  if (u_emissive <= 0.01 && pane >= 0.5)
    albedo *= surface_detail(v_world_position, eye_distance, normalize(v_normal).y);
  vec3 n = normalize(v_normal);
  if (u_has_texture == 1 && pane >= 0.5 && eye_distance < 60.0)
    n = bump_normal(n, v_world_position, texture_height,
                    0.12 * (1.0 - smoothstep(6.0, 30.0, eye_distance)));
  vec3 l = normalize(u_light_direction);
  vec3 v = normalize(u_eye - v_world_position);
  bool water = u_roughness < 0.15 && u_metallic > 0.3 && n.y > 0.9;
  if (water) {
    vec2 w = v_world_position.xz;
    float t = u_time;
    vec2 slope = vec2(sin(w.x * 0.9 + t * 1.3) + 0.6 * sin(w.x * 2.3 - w.y * 1.7 + t * 2.1),
                      sin(w.y * 1.1 - t * 1.1) + 0.6 * sin(w.y * 2.9 + w.x * 1.3 - t * 1.7));
    n = normalize(n + vec3(slope.x, 0.0, slope.y) * 0.035 * (1.0 - smoothstep(60.0, 300.0, eye_distance)));
  }
  vec3 h = normalize(l + v);
  float n_dot_l = max(dot(n, l), 0.0);
  float n_dot_v = max(dot(n, v), 0.0);
  float shininess = mix(512.0, 6.0, u_roughness);
  float spec = pow(max(dot(n, h), 0.0), shininess) * (1.0 - u_roughness * 0.7) * (0.5 + shininess / 400.0);
  vec3 specular_tint = mix(vec3(0.04), albedo, u_metallic);
  vec3 diffuse = albedo * (1.0 - u_metallic * 0.8);
  /* Light from the sky above and a bounce from the ground below. */
  vec3 sky_light = mix(u_sky_color, u_zenith_color, 0.45) + vec3(0.04);
  vec3 ground_light = vec3(0.42, 0.38, 0.33) * (0.35 + 0.65 * min(length(u_light_color), 1.4) / 1.4);
  vec3 ambient_light = mix(ground_light, sky_light, n.y * 0.5 + 0.5) * u_ambient * 1.0;
  float shadow = shadow_factor(n, l);
  vec3 color = albedo * ambient_light
             + (diffuse * n_dot_l + specular_tint * spec * n_dot_l) * u_light_color * shadow;
  /* Smooth surfaces mirror the sky, more at grazing angles. */
  if (u_metallic > 0.3 && u_roughness < 0.5) {
    vec3 r = reflect(-v, n);
    float fresnel = 0.04 + 0.96 * pow(1.0 - n_dot_v, 5.0);
    float strength = water ? 0.25 + 0.75 * fresnel : clamp(fresnel * 0.8 + u_metallic * 0.08 * (1.0 - u_roughness), 0.0, 0.5);
    color = mix(color, sky_reflection(r) * (water ? 1.0 : 0.85), strength);
    if (water) color += u_light_color * pow(max(dot(r, l), 0.0), 700.0) * 14.0 * shadow;
  }
  if (u_wall_ao > 0.0) {
    float up = max(v_uv.y * 3.2, 0.0);
    color *= mix(1.0 - 0.42 * u_wall_ao, 1.0, smoothstep(0.0, 1.8, up));
  }
  color += albedo * u_emissive;
  /* Window glass shows the sky and the sun, and at night the lit rooms. */
  if (u_has_texture == 1 && pane < 0.5 && u_window_glow >= 0.0) {
    vec3 r = reflect(-v, n);
    float fresnel = 0.05 + 0.5 * pow(1.0 - n_dot_v, 3.0);
    vec3 glass = mix(albedo * (ambient_light * 1.5 + 0.02), sky_reflection(r) * 0.55, fresnel);
    glass += u_light_color * pow(max(dot(r, l), 0.0), 160.0) * 2.5 * shadow;
    color = glass;
  }
  if (u_window_glow > 0.0 && pane < 0.5) {
    vec3 cell = floor(vec3(v_world_position.x + v_world_position.z, v_world_position.y, 0.0) * vec3(0.33, 0.3, 1.0));
    float hh = fract(sin(dot(cell.xy, vec2(12.9898, 78.233))) * 43758.5453);
    float lit = step(0.52, hh) * u_window_glow;
    color = mix(color, vec3(1.0, 0.78, 0.42) * (1.1 + 0.9 * hh), lit);
  }
  if (u_spot_intensity > 0.0) {
    vec3 to_point = v_world_position - u_spot_position;
    float d = length(to_point);
    vec3 ray = to_point / max(d, 0.001);
    float cone = smoothstep(0.55, 0.93, dot(ray, normalize(u_spot_direction)));
    float facing = max(dot(n, -ray), 0.0);
    float falloff = 1.0 / (1.0 + 0.012 * d * d);
    color += albedo * vec3(1.0, 0.94, 0.78) * u_spot_intensity * cone * facing * falloff;
  }
  /* Distance fog in the colour of the sky behind, brighter towards the sun. */
  float fog = clamp((eye_distance - u_fog_start) / (u_fog_end - u_fog_start), 0.0, 1.0);
  fog = fog * fog * (3.0 - 2.0 * fog);
  vec3 view_dir = normalize(v_world_position - u_eye);
  float toward_sun = max(dot(view_dir, l), 0.0);
  vec3 fog_color = u_sky_color + u_light_color * (pow(toward_sun, 6.0) * 0.12 + pow(toward_sun, 64.0) * 0.1);
  color = mix(color, fog_color, fog);
  if (u_debug_shadows == 1) color = vec3(shadow);
  out_color = vec4(u_srgb_out == 1 ? pow(color, vec3(1.0 / 2.2)) : color, 1.0);
}
