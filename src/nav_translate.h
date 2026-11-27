#ifndef NAV_TRANSLATE_H
#define NAV_TRANSLATE_H

/*
 * Android Auto 1.6+ navigation messages, translated into the 1.2 ones this
 * unit's receiver understands (issue #4). See nav_translate.c.
 */

/* Internal to the hook: never exported, so nothing in gal can bind to it. */
#define NAV_TRANSLATE_HIDDEN __attribute__((visibility("hidden")))

#define NAV_TRANSLATE_PASS (-1)  /* leave the message as it is */
#define NAV_TRANSLATE_DROP 0     /* nothing usable: do not route it */

/*
 * in/in_len: the whole message, starting with its two-byte id.
 * Returns NAV_TRANSLATE_PASS, NAV_TRANSLATE_DROP, or the length of the
 * translated message written to out (with its own id), never more than
 * in_len, so the caller can write it over the original.
 */
NAV_TRANSLATE_HIDDEN int nav_translate(const unsigned char *in, unsigned in_len,
                  unsigned char *out, unsigned out_cap);

/* Clear cached countdown when NavigationStatus or the AAP session changes. */
NAV_TRANSLATE_HIDDEN void nav_translate_reset(void);

/* Field availability in the input frame, without retaining names or ETA text. */
typedef struct nav_route_fields {
    unsigned steps, lanes, recommended_lanes, destinations, addresses;
    unsigned destination_distances, destination_meters, eta_texts, eta_seconds;
} nav_route_fields;

/* Last valid input's route fields and, if translated, the legacy output. */
typedef struct nav_translate_info {
    unsigned from_id, to_id;
    unsigned event, side, meters, seconds, road_len;
    int status, fields_valid;
    nav_route_fields route;
} nav_translate_info;
NAV_TRANSLATE_HIDDEN const nav_translate_info *nav_translate_last(void);

#endif
