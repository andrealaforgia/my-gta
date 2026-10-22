#include "navigation.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct {
  int to;
  int route;
  int forward; /* 1 along the route's points, 0 against them */
  float length;
} edge_t;

struct nav {
  const city_route_t* routes;
  size_t route_count;
  float* lengths;     /* per route, metres */
  float** cumulative; /* per route, distance at each point */
  int64_t* node_ids;  /* sorted */
  int node_count;
  int* first_edge; /* node_count + 1, into edges */
  edge_t* edges;
  int* edge_from; /* parallel to edges while building */
  /* Search state, reused. */
  float* distance;
  int* came_by; /* the edge that reached the node, or -1 */
  int* heap;
  int* heap_position;
  size_t heap_size;
};

static int compare_ids(const void* a, const void* b) {
  int64_t x = *(const int64_t*)a, y = *(const int64_t*)b;
  return x < y ? -1 : (x > y);
}

static int node_index(const nav_t* nav, int64_t id) {
  int low = 0, high = nav->node_count - 1;
  while (low <= high) {
    int middle = (low + high) / 2;
    if (nav->node_ids[middle] == id) return middle;
    if (nav->node_ids[middle] < id)
      low = middle + 1;
    else
      high = middle - 1;
  }
  return -1;
}

static bool usable(const city_route_t* route, float length) {
  /* Cars cannot turn into anything under six metres, so neither does the
   * satnav. */
  return route->drivable && route->point_count >= 2 && length >= 6.0f;
}

void nav_destroy(nav_t* nav) {
  if (!nav) return;
  if (nav->cumulative) {
    for (size_t r = 0; r < nav->route_count; r++) free(nav->cumulative[r]);
  }
  free(nav->cumulative);
  free(nav->lengths);
  free(nav->node_ids);
  free(nav->first_edge);
  free(nav->edges);
  free(nav->edge_from);
  free(nav->distance);
  free(nav->came_by);
  free(nav->heap);
  free(nav->heap_position);
  free(nav);
}

nav_t* nav_create(const city_route_t* routes, size_t route_count) {
  nav_t* nav = calloc(1, sizeof(nav_t));
  if (!nav) return NULL;
  nav->routes = routes;
  nav->route_count = route_count;
  nav->lengths = calloc(route_count ? route_count : 1, sizeof(float));
  nav->cumulative = calloc(route_count ? route_count : 1, sizeof(float*));
  if (!nav->lengths || !nav->cumulative) {
    nav_destroy(nav);
    return NULL;
  }
  size_t usable_count = 0;
  for (size_t r = 0; r < route_count; r++) {
    const city_route_t* route = &routes[r];
    if (route->point_count < 2) continue;
    nav->cumulative[r] = calloc(route->point_count, sizeof(float));
    if (!nav->cumulative[r]) {
      nav_destroy(nav);
      return NULL;
    }
    for (size_t k = 1; k < route->point_count; k++)
      nav->cumulative[r][k] =
          nav->cumulative[r][k - 1] +
          hypotf(route->points[2 * k] - route->points[2 * k - 2],
                 route->points[2 * k + 1] - route->points[2 * k - 1]);
    nav->lengths[r] = nav->cumulative[r][route->point_count - 1];
    if (usable(route, nav->lengths[r])) usable_count++;
  }
  if (usable_count == 0) {
    nav_destroy(nav);
    return NULL;
  }
  nav->node_ids = malloc(2 * usable_count * sizeof(int64_t));
  if (!nav->node_ids) {
    nav_destroy(nav);
    return NULL;
  }
  size_t n = 0;
  for (size_t r = 0; r < route_count; r++)
    if (usable(&routes[r], nav->lengths[r])) {
      nav->node_ids[n++] = routes[r].start_node;
      nav->node_ids[n++] = routes[r].end_node;
    }
  qsort(nav->node_ids, n, sizeof(int64_t), compare_ids);
  size_t unique = 0;
  for (size_t i = 0; i < n; i++)
    if (i == 0 || nav->node_ids[i] != nav->node_ids[i - 1])
      nav->node_ids[unique++] = nav->node_ids[i];
  nav->node_count = (int)unique;
  nav->first_edge = calloc((size_t)nav->node_count + 1, sizeof(int));
  nav->edges = malloc(2 * usable_count * sizeof(edge_t));
  nav->edge_from = malloc(2 * usable_count * sizeof(int));
  nav->distance = malloc((size_t)nav->node_count * sizeof(float));
  nav->came_by = malloc((size_t)nav->node_count * sizeof(int));
  nav->heap = malloc((size_t)nav->node_count * sizeof(int));
  nav->heap_position = malloc((size_t)nav->node_count * sizeof(int));
  if (!nav->first_edge || !nav->edges || !nav->edge_from || !nav->distance ||
      !nav->came_by || !nav->heap || !nav->heap_position) {
    nav_destroy(nav);
    return NULL;
  }
  /* Edges in compressed rows: count, then place. */
  size_t edge_total = 0;
  for (int pass = 0; pass < 2; pass++) {
    if (pass == 1) {
      for (int i = 0; i < nav->node_count; i++)
        nav->first_edge[i + 1] += nav->first_edge[i];
    }
    int* fill = pass == 1 ? calloc((size_t)nav->node_count, sizeof(int)) : NULL;
    for (size_t r = 0; r < route_count; r++) {
      const city_route_t* route = &routes[r];
      if (!usable(route, nav->lengths[r])) continue;
      int a = node_index(nav, route->start_node);
      int b = node_index(nav, route->end_node);
      for (int forward = 1; forward >= 0; forward--) {
        if (!forward && route->oneway) continue;
        int from = forward ? a : b, to = forward ? b : a;
        if (pass == 0) {
          nav->first_edge[from + 1]++;
          edge_total++;
        } else {
          edge_t e = {to, (int)r, forward, nav->lengths[r]};
          int slot = nav->first_edge[from] + fill[from]++;
          nav->edges[slot] = e;
          nav->edge_from[slot] = from;
        }
      }
    }
    free(fill);
  }
  (void)edge_total;
  return nav;
}

/* ---- heap ---- */

static void heap_swap(nav_t* nav, size_t i, size_t j) {
  int a = nav->heap[i], b = nav->heap[j];
  nav->heap[i] = b, nav->heap[j] = a;
  nav->heap_position[b] = (int)i, nav->heap_position[a] = (int)j;
}

static void heap_up(nav_t* nav, size_t i) {
  while (i > 0) {
    size_t parent = (i - 1) / 2;
    if (nav->distance[nav->heap[parent]] <= nav->distance[nav->heap[i]]) break;
    heap_swap(nav, i, parent);
    i = parent;
  }
}

static void heap_down(nav_t* nav, size_t i) {
  for (;;) {
    size_t l = 2 * i + 1, r = l + 1, smallest = i;
    if (l < nav->heap_size &&
        nav->distance[nav->heap[l]] < nav->distance[nav->heap[smallest]])
      smallest = l;
    if (r < nav->heap_size &&
        nav->distance[nav->heap[r]] < nav->distance[nav->heap[smallest]])
      smallest = r;
    if (smallest == i) break;
    heap_swap(nav, i, smallest);
    i = smallest;
  }
}

static void relax(nav_t* nav, int node, float distance, int by) {
  if (distance >= nav->distance[node]) return;
  nav->distance[node] = distance;
  nav->came_by[node] = by;
  if (nav->heap_position[node] < 0) {
    nav->heap[nav->heap_size] = node;
    nav->heap_position[node] = (int)nav->heap_size++;
  }
  heap_up(nav, (size_t)nav->heap_position[node]);
}

/* ---- snapping a point to a road ---- */

typedef struct {
  int route;
  float along; /* metres from the route's first point */
  float off;   /* metres from the road */
} snap_t;

static snap_t snap_route(const nav_t* nav, size_t r, float x, float z) {
  snap_t best = {(int)r, 0, 1e30f};
  const city_route_t* route = &nav->routes[r];
  for (size_t k = 1; k < route->point_count; k++) {
    float ax = route->points[2 * k - 2], az = route->points[2 * k - 1];
    float bx = route->points[2 * k], bz = route->points[2 * k + 1];
    float dx = bx - ax, dz = bz - az, l2 = dx * dx + dz * dz;
    float t = l2 > 1e-6f ? ((x - ax) * dx + (z - az) * dz) / l2 : 0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    float off = hypotf(x - (ax + dx * t), z - (az + dz * t));
    if (off < best.off) {
      best.off = off;
      best.along = nav->cumulative[r][k - 1] +
                   t * (nav->cumulative[r][k] - nav->cumulative[r][k - 1]);
    }
  }
  return best;
}

static snap_t snap(const nav_t* nav, float x, float z) {
  snap_t best = {-1, 0, 1e30f};
  for (size_t r = 0; r < nav->route_count; r++) {
    if (!usable(&nav->routes[r], nav->lengths[r])) continue;
    snap_t candidate = snap_route(nav, r, x, z);
    if (candidate.off < best.off) best = candidate;
  }
  return best;
}

/* Runs the search to the end from whatever nodes have been given a distance
 * with relax(). */
static void search(nav_t* nav) {
  while (nav->heap_size > 0) {
    int node = nav->heap[0];
    heap_swap(nav, 0, nav->heap_size - 1);
    nav->heap_size--;
    nav->heap_position[node] = -2;
    if (nav->heap_size > 0) heap_down(nav, 0);
    for (int e = nav->first_edge[node]; e < nav->first_edge[node + 1]; e++) {
      int to = nav->edges[e].to;
      if (nav->heap_position[to] == -2) continue;
      relax(nav, to, nav->distance[node] + nav->edges[e].length, e);
    }
  }
}

static void reset_search(nav_t* nav) {
  for (int i = 0; i < nav->node_count; i++) {
    nav->distance[i] = 1e30f;
    nav->came_by[i] = -1;
    nav->heap_position[i] = -1;
  }
  nav->heap_size = 0;
}

/* After a search: the road to arrive at (x, z) by. A spot may lie near a
 * road that cannot be reached (a one-way spur, an island), so among the
 * roads within 40 m of the nearest the cheapest to reach is taken, the walk
 * from its road counting double. `from`, if given, is where the search began:
 * its own road can be driven along directly. Returns route -1 if none. */
static snap_t best_target(const nav_t* nav, float x, float z,
                          const snap_t* from) {
  snap_t nearest = snap(nav, x, z);
  snap_t best = {-1, 0, 1e30f};
  if (nearest.route < 0) return best;
  float best_cost = 1e30f;
  for (size_t r = 0; r < nav->route_count; r++) {
    const city_route_t* route = &nav->routes[r];
    if (!usable(route, nav->lengths[r])) continue;
    snap_t c = snap_route(nav, r, x, z);
    if (c.off > nearest.off + 40.0f) continue;
    float cost = nav->distance[node_index(nav, route->start_node)] + c.along;
    if (!route->oneway)
      cost = fminf(cost, nav->distance[node_index(nav, route->end_node)] +
                             (nav->lengths[r] - c.along));
    if (from && (int)r == from->route &&
        (c.along >= from->along || !route->oneway))
      cost = fminf(cost, fabsf(c.along - from->along));
    cost += 2.0f * c.off;
    if (cost < best_cost) best_cost = cost, best = c;
  }
  return best_cost >= 1e29f ? (snap_t){-1, 0, 1e30f} : best;
}

/* ---- the polyline ---- */

typedef struct {
  float* out;
  size_t max, count;
} writer_t;

static void put(writer_t* w, float x, float z) {
  if (w->count >= w->max) return;
  if (w->count > 0 && fabsf(w->out[2 * w->count - 2] - x) < 0.01f &&
      fabsf(w->out[2 * w->count - 1] - z) < 0.01f)
    return;
  w->out[2 * w->count] = x;
  w->out[2 * w->count + 1] = z;
  w->count++;
}

static void point_at(const nav_t* nav, int route, float along, float* x,
                     float* z) {
  const city_route_t* r = &nav->routes[route];
  const float* cum = nav->cumulative[route];
  size_t k = 1;
  while (k + 1 < r->point_count && cum[k] < along) k++;
  float span = cum[k] - cum[k - 1];
  float t = span > 1e-6f ? (along - cum[k - 1]) / span : 0;
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  *x = r->points[2 * k - 2] + (r->points[2 * k] - r->points[2 * k - 2]) * t;
  *z = r->points[2 * k - 1] + (r->points[2 * k + 1] - r->points[2 * k - 1]) * t;
}

/* The route's points between two distances along it, in the order of travel. */
static void slice(const nav_t* nav, writer_t* w, int route, float from,
                  float to) {
  const city_route_t* r = &nav->routes[route];
  const float* cum = nav->cumulative[route];
  float x, z;
  point_at(nav, route, from, &x, &z);
  put(w, x, z);
  if (from <= to) {
    for (size_t k = 0; k < r->point_count; k++)
      if (cum[k] > from && cum[k] < to)
        put(w, r->points[2 * k], r->points[2 * k + 1]);
  } else {
    for (size_t k = r->point_count; k-- > 0;)
      if (cum[k] < from && cum[k] > to)
        put(w, r->points[2 * k], r->points[2 * k + 1]);
  }
  point_at(nav, route, to, &x, &z);
  put(w, x, z);
}

size_t nav_path(nav_t* nav, float from_x, float from_z, float to_x, float to_z,
                float* out, size_t max_points, float* length) {
  if (!nav || max_points < 2) return 0;
  snap_t a = snap(nav, from_x, from_z);
  if (a.route < 0) return 0;
  const city_route_t* ra = &nav->routes[a.route];
  float la = nav->lengths[a.route];
  reset_search(nav);
  /* Leave the start's road by either end it allows. */
  int end_a = node_index(nav, ra->end_node),
      start_a = node_index(nav, ra->start_node);
  relax(nav, end_a, la - a.along, -1);
  if (!ra->oneway) relax(nav, start_a, a.along, -1);
  search(nav);
  snap_t b = best_target(nav, to_x, to_z, &a);
  if (b.route < 0) return 0;
  const city_route_t* rb = &nav->routes[b.route];
  float lb = nav->lengths[b.route];
  /* Straight along one road, if it can be driven that way. */
  float direct = 1e30f;
  if (a.route == b.route && (b.along >= a.along || !ra->oneway))
    direct = fabsf(b.along - a.along);
  int goal_from_start = node_index(nav, rb->start_node);
  int goal_from_end = node_index(nav, rb->end_node);
  /* Arrive by the start of the target road, or by its end against the flow
   * if it is two-way. */
  float via_start = nav->distance[goal_from_start] + b.along;
  float via_end =
      rb->oneway ? 1e30f : nav->distance[goal_from_end] + (lb - b.along);
  float best = fminf(direct, fminf(via_start, via_end));
  if (best >= 1e29f) return 0;
  if (length) *length = best;
  writer_t w = {out, max_points, 0};
  if (best == direct) {
    slice(nav, &w, a.route, a.along, b.along);
    return w.count;
  }
  int goal = best == via_start ? goal_from_start : goal_from_end;
  /* Back along the edges to the node the start's road leaves by. */
  enum { MAX_HOPS = 4096 };
  int hops[MAX_HOPS];
  int hop_count = 0;
  int node = goal;
  while (nav->came_by[node] >= 0 && hop_count < MAX_HOPS) {
    int e = nav->came_by[node];
    hops[hop_count++] = e;
    node = nav->edge_from[e];
  }
  /* `node` is where the start's road ends or starts. */
  if (node == end_a)
    slice(nav, &w, a.route, a.along, la);
  else
    slice(nav, &w, a.route, a.along, 0);
  for (int i = hop_count - 1; i >= 0; i--) {
    const edge_t* e = &nav->edges[hops[i]];
    if (e->forward)
      slice(nav, &w, e->route, 0, e->length);
    else
      slice(nav, &w, e->route, e->length, 0);
  }
  if (best == via_start)
    slice(nav, &w, b.route, 0, b.along);
  else
    slice(nav, &w, b.route, lb, b.along);
  return w.count;
}

bool nav_next_hop(nav_t* nav, int route, int forward, float to_x, float to_z,
                  int* next_route, int* next_forward) {
  if (!nav || route < 0 || (size_t)route >= nav->route_count) return false;
  const city_route_t* here = &nav->routes[route];
  int origin = node_index(nav, forward ? here->end_node : here->start_node);
  if (origin < 0) return false;
  reset_search(nav);
  relax(nav, origin, 0, -1);
  search(nav);
  snap_t b = best_target(nav, to_x, to_z, NULL);
  if (b.route < 0 || b.route == route) return false;
  const city_route_t* rb = &nav->routes[b.route];
  float lb = nav->lengths[b.route];
  int goal_from_start = node_index(nav, rb->start_node);
  int goal_from_end = node_index(nav, rb->end_node);
  float via_start = nav->distance[goal_from_start] + b.along;
  float via_end =
      rb->oneway ? 1e30f : nav->distance[goal_from_end] + (lb - b.along);
  if (fminf(via_start, via_end) >= 1e29f) return false;
  int goal = via_start <= via_end ? goal_from_start : goal_from_end;
  if (nav->came_by[goal] < 0) {
    /* The target's road leaves from where we are. */
    *next_route = b.route;
    *next_forward = goal == goal_from_start ? 1 : 0;
    return true;
  }
  int node = goal, first = -1, hops = 0;
  while (nav->came_by[node] >= 0 && hops++ < 100000) {
    first = nav->came_by[node];
    node = nav->edge_from[first];
  }
  if (first < 0) return false;
  *next_route = nav->edges[first].route;
  *next_forward = nav->edges[first].forward;
  return true;
}
