/*
 * nav_translate.c -- Android Auto 1.6+ navigation, spoken to a 1.2 receiver.
 *
 * At protocol 1.6 and above (GAL_DUALSCREEN_AAP_MINOR) Google Maps stops
 * sending 0x8004 NavigationNextTurnEvent / 0x8005 NavigationNextTurnDistance
 * and sends 0x8006 NavigationState / 0x8007 NavigationCurrentPosition
 * instead. This unit's receiver was built against 1.2, answers the new pair
 * with -253, and the turn-arrows jar never hears of the route (#4). That is
 * why the shipped setting is 1.5.
 *
 * Here the new pair is decoded and re-encoded as the old one, in place, so
 * the stock NavigationStatusEndpoint and everything behind it (the arrows
 * jar, the cockpit's own turn pictograms) see what they always saw.
 *
 * The 1.7 schema and the manoeuvre table come from kkrvch's
 * mib-std2-kombibridge (shim/src/navproto.h), which reads the same messages
 * on MIB2 Standard units; its author invited reuse in that project's issue
 * #5. The legacy field numbers are this unit's own, read from the
 * k*FieldNumber symbols in libautoreceiver.so:
 *
 *   0x8004 NavigationNextTurnEvent  road=1 (string) turnSide=2 event=3
 *                                   image=4 turnNumber=5 turnAngle=6
 *   0x8005 NavigationNextTurnDistanceEvent  distanceMeters=1 timeToTurnSeconds=2
 *   0x8003 NavigationStatus         status=1 (0 unavailable, 1 active, 2 inactive)
 *
 * The parser also reports field availability for hardware validation.
 * The legacy messages cannot carry lanes, the address or arrival estimates.
 */
#include <limits.h>
#include <string.h>
#include "nav_translate.h"

static nav_translate_info g_last;

const nav_translate_info *nav_translate_last(void) { return &g_last; }

/* ---- a bounded protobuf reader ------------------------------------------ */
typedef struct { const unsigned char *p, *e; int invalid; } pb;

static unsigned pb_var(pb *s)
{
    unsigned v = 0u;
    unsigned i;
    int wide = 0;
    for (i = 0; i < 10u && s->p < s->e; ++i) {
        unsigned char b = *s->p++;
        unsigned bits = b & 0x7fu;
        if (i == 9u && b > 1u) break;
        if (i < 5u) v |= bits << (7u * i);
        if ((i == 4u && bits > 15u) || (i > 4u && bits != 0u)) wide = 1;
        if (!(b & 0x80u)) return wide ? UINT_MAX : v;
    }
    s->invalid = 1;
    return UINT_MAX;
}

static int pb_tag(pb *s, unsigned *f, unsigned *w)
{
    unsigned t;
    if (s->invalid || s->p >= s->e) return 0;
    t = pb_var(s);
    if (t < 8u || t == UINT_MAX) {
        s->invalid = 1;
        return 0;
    }
    *f = t >> 3;
    *w = t & 7u;
    return 1;
}

static int pb_sub(pb *s, pb *o)
{
    unsigned n = pb_var(s);
    if (s->invalid || n == UINT_MAX || n > (unsigned)(s->e - s->p)) {
        s->invalid = 1;
        return 0;
    }
    o->p = s->p;
    o->e = s->p + n;
    o->invalid = 0;
    s->p += n;
    return 1;
}

static void pb_skip(pb *s, unsigned w)
{
    unsigned left = (unsigned)(s->e - s->p);
    pb ignored;
    if (w == 0u) { (void)pb_var(s); return; }
    if (w == 2u) { (void)pb_sub(s, &ignored); return; }
    if (w == 5u && left >= 4u) { s->p += 4u; return; }
    if (w == 1u && left >= 8u) { s->p += 8u; return; }
    s->invalid = 1;
}

/* A string field, cut to fit, without the newlines a cockpit line cannot show. */
static unsigned pb_str(pb *s, char *dst, unsigned cap)
{
    pb value;
    unsigned j = 0u;
    if (!pb_sub(s, &value)) return 0u;
    while (value.p < value.e && j + 1u < cap) {
        char c = (char)*value.p++;
        if (c == '\n' || c == '\r' || c == '\t') c = ' ';
        dst[j++] = c;
    }
    dst[j] = '\0';
    return j;
}

static int pb_has_text(pb s, unsigned field)
{
    unsigned f, w;
    int present = 0;
    while (pb_tag(&s, &f, &w)) {
        if (f == field && w == 2u) {
            pb value;
            if (!pb_sub(&s, &value)) break;
            present = value.p != value.e;
        } else {
            pb_skip(&s, w);
        }
    }
    return s.invalid ? -1 : present;
}

/* ---- a protobuf writer -------------------------------------------------- */
typedef struct { unsigned char *p; unsigned n, cap; int full; } pw;

static void pw_byte(pw *o, unsigned char b)
{
    if (o->n < o->cap) o->p[o->n++] = b;
    else o->full = 1;
}

static void pw_var(pw *o, unsigned v)
{
    while (v >= 0x80u) { pw_byte(o, (unsigned char)(v | 0x80u)); v >>= 7; }
    pw_byte(o, (unsigned char)v);
}

static void pw_uint(pw *o, unsigned field, unsigned v)
{
    pw_var(o, field << 3);
    pw_var(o, v);
}

static void pw_string(pw *o, unsigned field, const char *str, unsigned len)
{
    unsigned i;
    pw_var(o, (field << 3) | 2u);
    pw_var(o, len);
    for (i = 0u; i < len; ++i) pw_byte(o, (unsigned char)str[i]);
}

/* ---- 1.7 NavigationType (1..50) -> the 1.2 event and side ----------------
 * kombibridge's table, with its jar-internal numbers for the last three
 * events changed to this receiver's wire values: FERRY_BOAT 16,
 * FERRY_TRAIN 17, DESTINATION 19. Side: 1 left, 2 right, 3 unspecified.
 * Counter-clockwise roundabouts map to RIGHT, as the 1.2 feed sent them. */
#define MV(e, s) (unsigned char)(((e) << 2) | (s))
#define N_MANEUVER 51
static const unsigned char k_maneuver[N_MANEUVER] = {
    MV(0, 3),                                   /*  0 invalid */
    MV(1, 3), MV(2, 3),                         /*  1 DEPART, 2 NAME_CHANGE */
    MV(9, 1), MV(9, 2),                         /*  3 KEEP_LEFT/RIGHT -> FORK */
    MV(3, 1), MV(3, 2),                         /*  5 TURN_SLIGHT_L/R */
    MV(4, 1), MV(4, 2),                         /*  7 TURN_NORMAL_L/R */
    MV(5, 1), MV(5, 2),                         /*  9 TURN_SHARP_L/R */
    MV(6, 1), MV(6, 2),                         /* 11 U_TURN_L/R */
    MV(7, 1), MV(7, 2), MV(7, 1), MV(7, 2),     /* 13 ON_RAMP slight, normal */
    MV(7, 1), MV(7, 2), MV(6, 1), MV(6, 2),     /* 17 ON_RAMP sharp, u-turn */
    MV(8, 1), MV(8, 2), MV(8, 1), MV(8, 2),     /* 21 OFF_RAMP slight, normal */
    MV(9, 1), MV(9, 2),                         /* 25 FORK_L/R */
    MV(10, 1), MV(10, 2), MV(10, 3),            /* 27 MERGE_L/R/unspecified */
    MV(0, 3), MV(0, 3),                         /* 30, 31 unassigned */
    MV(13, 1), MV(13, 1), MV(13, 2), MV(13, 2), /* 32 ROUNDABOUT_ENTER_AND_EXIT CW/CCW */
    MV(14, 3),                                  /* 36 STRAIGHT */
    MV(16, 3), MV(17, 3),                       /* 37 FERRY_BOAT, FERRY_TRAIN */
    MV(19, 3), MV(19, 3), MV(19, 1), MV(19, 2), /* 39 DESTINATION{,_STRAIGHT,_LEFT,_RIGHT} */
    MV(11, 1), MV(12, 1), MV(11, 2), MV(12, 2), /* 43 ROUNDABOUT ENTER/EXIT CW, CCW */
    MV(16, 1), MV(16, 2), MV(17, 1), MV(17, 2)  /* 47 FERRY_BOAT_L/R, FERRY_TRAIN_L/R */
};

#define ROAD_MAX 64u

typedef struct {
    unsigned event, side, number, angle, road_len;
    char road[ROAD_MAX];
} turn;

static int parse_maneuver(pb s, turn *t)
{
    unsigned f, w, v;
    while (pb_tag(&s, &f, &w)) {
        if (f == 1u && w == 0u) {
            unsigned char m;
            v = pb_var(&s);
            m = v < N_MANEUVER ? k_maneuver[v] : k_maneuver[0];
            t->event = (unsigned)(m >> 2);
            t->side = (unsigned)(m & 3u);
        } else if (f == 2u && w == 0u) {
            v = pb_var(&s);
            t->number = (v >= 1u && v <= 99u) ? v : 0u;
        } else if (f == 3u && w == 0u) {
            v = pb_var(&s);
            t->angle = (v >= 1u && v <= 360u) ? v : 0u;
        } else {
            pb_skip(&s, w);
        }
    }
    return !s.invalid;
}

static int parse_lane(pb s, nav_route_fields *route)
{
    unsigned f, w;
    int has_shape = 0, recommended = 0;
    while (pb_tag(&s, &f, &w)) {
        pb direction;
        unsigned df, dw, shape = 0u, highlight = 0u;
        if (f != 1u || w != 2u) { pb_skip(&s, w); continue; }
        if (!pb_sub(&s, &direction)) break;
        while (pb_tag(&direction, &df, &dw)) {
            if (df == 1u && dw == 0u) shape = pb_var(&direction);
            else if (df == 2u && dw == 0u) highlight = pb_var(&direction);
            else pb_skip(&direction, dw);
        }
        if (direction.invalid) return 0;
        if (shape >= 1u && shape <= 9u) {
            has_shape = 1;
            if (highlight) recommended = 1;
        }
    }
    if (s.invalid) return 0;
    route->lanes += has_shape;
    route->recommended_lanes += recommended;
    return 1;
}

/* NavigationStep { maneuver=1; instruction=2 {text=1}; lanes=3; road_info=4 {road_names=1} }.
 * The road name wins over the spoken instruction, whatever order they come in. */
static int parse_step(pb s, turn *t, nav_route_fields *route)
{
    char cue[ROAD_MAX];
    unsigned cue_len = 0u, f, w;
    cue[0] = '\0';
    t->event = 0u; t->side = 3u; t->number = 0u; t->angle = 0u;
    t->road[0] = '\0'; t->road_len = 0u;
    while (pb_tag(&s, &f, &w)) {
        pb d;
        unsigned df, dw;
        if (w != 2u) { pb_skip(&s, w); continue; }
        if (!pb_sub(&s, &d)) break;
        if (f == 1u) {
            if (!parse_maneuver(d, t)) return 0;
        } else if (f == 2u) {
            while (pb_tag(&d, &df, &dw)) {
                if (df == 1u && dw == 2u) cue_len = pb_str(&d, cue, sizeof cue);
                else pb_skip(&d, dw);
            }
        } else if (f == 3u) {
            if (!parse_lane(d, route)) return 0;
        } else if (f == 4u) {
            while (pb_tag(&d, &df, &dw)) {
                if (df == 1u && dw == 2u && t->road_len == 0u)
                    t->road_len = pb_str(&d, t->road, sizeof t->road);
                else pb_skip(&d, dw);
            }
        }
        if (d.invalid) return 0;
    }
    if (t->road_len == 0u && cue_len > 0u) {
        memcpy(t->road, cue, cue_len + 1u);
        t->road_len = cue_len;
    }
    return !s.invalid;
}

/* Distances survive between messages: an "unknown" 0x8007 keeps the last
 * good countdown on the cockpit instead of blanking it. */
static int g_meters = -1;

void nav_translate_reset(void)
{
    g_meters = -1;
}

static int parse_meters(pb s, int *meters)
{
    unsigned f, w;
    while (pb_tag(&s, &f, &w)) {
        if (f == 1u && w == 0u) {
            unsigned value = pb_var(&s);
            *meters = value <= 20000000u ? (int)value : -1;
        } else {
            pb_skip(&s, w);
        }
    }
    return !s.invalid;
}

/* Both position entries contain a distance. Field 2 is step seconds or
 * destination ETA text depending on the entry, and field 3 is arrival seconds. */
static int parse_position_entry(pb s, int destination, int *meters, int *seconds,
                                int *eta_text)
{
    unsigned f, w;
    while (pb_tag(&s, &f, &w)) {
        pb value;
        if (f == 1u && w == 2u) {
            if (!pb_sub(&s, &value) || !parse_meters(value, meters)) return 0;
        } else if (f == (destination ? 3u : 2u) && w == 0u) {
            unsigned v = pb_var(&s);
            *seconds = v <= 1000000u ? (int)v : -1;
        } else if (destination && f == 2u && w == 2u) {
            if (!pb_sub(&s, &value)) return 0;
            *eta_text = value.p != value.e;
        } else {
            pb_skip(&s, w);
        }
    }
    return !s.invalid;
}

static int parse_position(pb s, int *step_meters, int *step_seconds,
                          int *have_step, nav_route_fields *route)
{
    unsigned f, w;
    while (pb_tag(&s, &f, &w)) {
        pb entry;
        if ((f != 1u && f != 2u) || w != 2u) { pb_skip(&s, w); continue; }
        if (!pb_sub(&s, &entry)) break;
        if (f == 1u) {
            if (!parse_position_entry(entry, 0, step_meters, step_seconds, NULL)) return 0;
            *have_step = 1;
        } else {
            int meters = -1, seconds = -1, eta_text = 0;
            if (!parse_position_entry(entry, 1, &meters, &seconds, &eta_text)) return 0;
            ++route->destination_distances;
            route->destination_meters += meters >= 0;
            route->eta_texts += eta_text;
            route->eta_seconds += seconds >= 0;
        }
    }
    return !s.invalid;
}

int nav_translate(const unsigned char *in, unsigned in_len,
                  unsigned char *out, unsigned out_cap)
{
    unsigned id, f, w;
    pb s;
    pw o;

    if (in == NULL || in_len < 2u) return NAV_TRANSLATE_PASS;
    id = ((unsigned)in[0] << 8) | in[1];
    s.p = in + 2;
    s.e = in + in_len;
    o.p = out;
    o.n = 0u;
    o.cap = out_cap < in_len ? out_cap : in_len;   /* must fit over the original */
    o.full = 0;
    s.invalid = 0;
    memset(&g_last, 0, sizeof g_last);
    g_last.from_id = id;
    g_last.status = -1;

    if (id == 0x8006u) {
        /* NavigationState { repeated NavigationStep steps=1; ... }: the first step is the next turn */
        turn t = {0};
        nav_route_fields route = {0};
        while (pb_tag(&s, &f, &w)) {
            pb entry;
            if ((f != 1u && f != 2u) || w != 2u) { pb_skip(&s, w); continue; }
            if (!pb_sub(&s, &entry)) break;
            if (f == 1u) {
                if (route.steps == 0u && !parse_step(entry, &t, &route)) return NAV_TRANSLATE_DROP;
                ++route.steps;
            } else {
                int address = pb_has_text(entry, 1u);
                if (address < 0) return NAV_TRANSLATE_DROP;
                ++route.destinations;
                route.addresses += address;
            }
        }
        if (s.invalid) return NAV_TRANSLATE_DROP;
        g_last.route = route;
        g_last.fields_valid = 1;
        if (route.steps == 0u) return NAV_TRANSLATE_DROP;
        pw_byte(&o, 0x80u);
        pw_byte(&o, 0x04u);
        if (t.road_len > 0u) pw_string(&o, 1u, t.road, t.road_len);
        pw_uint(&o, 2u, t.side);
        pw_uint(&o, 3u, t.event);
        if (t.number > 0u) pw_uint(&o, 5u, t.number);
        if (t.angle > 0u) pw_uint(&o, 6u, t.angle);
        if (o.full) return NAV_TRANSLATE_PASS;
        g_last.to_id = 0x8004u;
        g_last.event = t.event;
        g_last.side = t.side;
        g_last.road_len = t.road_len;
        return (int)o.n;
    }
    if (id == 0x8007u) {
        nav_route_fields route = {0};
        int meters = -1, seconds = -1, have_step = 0;
        unsigned legacy_seconds;
        if (!parse_position(s, &meters, &seconds, &have_step, &route)) return NAV_TRANSLATE_DROP;
        g_last.route = route;
        g_last.fields_valid = 1;
        if (!have_step) return NAV_TRANSLATE_DROP;
        if (meters < 0) meters = g_meters;
        if (meters < 0) return NAV_TRANSLATE_DROP;
        /* A phone may omit time even when it sends a usable distance. */
        legacy_seconds = seconds < 0 ? 0u : (unsigned)seconds;
        pw_byte(&o, 0x80u);
        pw_byte(&o, 0x05u);
        pw_uint(&o, 1u, (unsigned)meters);
        pw_uint(&o, 2u, legacy_seconds);
        if (o.full) return NAV_TRANSLATE_PASS;
        g_meters = meters;
        g_last.to_id = 0x8005u;
        g_last.meters = (unsigned)meters;
        g_last.seconds = legacy_seconds;
        return (int)o.n;
    }
    if (id == 0x8003u) {
        /* 1.7 adds status 3, REROUTING, which the 1.2 enum stops short of.
         * During a recalculation the route is still on: call it ACTIVE. */
        unsigned status = UINT_MAX;
        while (pb_tag(&s, &f, &w)) {
            if (f == 1u && w == 0u) status = pb_var(&s);
            else pb_skip(&s, w);
        }
        if (s.invalid) return NAV_TRANSLATE_DROP;
        if (status == 0u || status == 2u || status == 3u) nav_translate_reset();
        if (status != 3u) return NAV_TRANSLATE_PASS;
        pw_byte(&o, 0x80u);
        pw_byte(&o, 0x03u);
        pw_uint(&o, 1u, 1u);
        if (o.full) return NAV_TRANSLATE_PASS;
        g_last.to_id = 0x8003u;
        g_last.status = 1;
        return (int)o.n;
    }
    return NAV_TRANSLATE_PASS;
}
