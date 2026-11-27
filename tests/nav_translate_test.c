/* Host test for src/nav_translate.c: cc -I src tests/nav_translate_test.c src/nav_translate.c */
#include <stdio.h>
#include <string.h>
#include "nav_translate.h"

static int failures;

static void check(const char *name, int ok)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) ++failures;
}

static void expect(const char *name, const unsigned char *in, unsigned in_len,
                   int want_len, const unsigned char *want)
{
    unsigned char out[512];
    int n = nav_translate(in, in_len, out, sizeof out);
    int ok = n == want_len && (n <= 0 || memcmp(out, want, (size_t)n) == 0);
    printf("%s %s (got %d)\n", ok ? "PASS" : "FAIL", name, n);
    if (!ok) {
        int i;
        ++failures;
        for (i = 0; i < n; ++i) printf("%02x ", out[i]);
        printf("\n");
    }
}

int main(void)
{
    /* 0x8006 NavigationState { steps { maneuver { type 8 (TURN_NORMAL_RIGHT) }
     *   instruction { text "Turn right" } road_info { road_names "Main St" } } } */
    static const unsigned char state[] = {
        0x80, 0x06,
        0x0a, 0x1d,                                    /* steps, 29 bytes */
        0x0a, 0x02, 0x08, 0x08,                        /* maneuver { type 8 } */
        0x12, 0x0c, 0x0a, 0x0a, 'T','u','r','n',' ','r','i','g','h','t',
        0x22, 0x09, 0x0a, 0x07, 'M','a','i','n',' ','S','t',
    };
    /* -> 0x8004 { road "Main St", turnSide 2, event 4 } */
    static const unsigned char turn[] = {
        0x80, 0x04, 0x0a, 0x07, 'M','a','i','n',' ','S','t', 0x10, 0x02, 0x18, 0x04,
    };
    expect("0x8006 turn right -> 0x8004", state, sizeof state, (int)sizeof turn, turn);

    /* roundabout, exit 2, angle 90, instruction only (no road_info) */
    static const unsigned char rbt[] = {
        0x80, 0x06,
        0x0a, 0x0c,
        0x0a, 0x06, 0x08, 0x20, 0x10, 0x02, 0x18, 0x5a, /* type 32, exit 2, angle 90 */
        0x12, 0x02, 0x0a, 0x00,                         /* empty instruction */
    };
    static const unsigned char rbt_out[] = { 0x80, 0x04, 0x10, 0x01, 0x18, 0x0d, 0x28, 0x02, 0x30, 0x5a };
    expect("0x8006 roundabout -> 0x8004 with number/angle", rbt, sizeof rbt, (int)sizeof rbt_out, rbt_out);

    static const unsigned char nostep[] = {
        0x80, 0x06, 0x12, 0x02, 0x0a, 0x00, 0x12, 0x03, 0x0a, 0x01, 'A',
    };
    expect("0x8006 destinations only produce no legacy turn",
           nostep, sizeof nostep, NAV_TRANSLATE_DROP, NULL);
    check("0x8006 reports both destinations and the nonempty address",
          nav_translate_last()->fields_valid && nav_translate_last()->route.steps == 0 &&
          nav_translate_last()->route.destinations == 2 && nav_translate_last()->route.addresses == 1);

    static const unsigned char arrive[] = { 0x80, 0x06, 0x0a, 0x04, 0x0a, 0x02, 0x08, 0x2a };
    static const unsigned char arrive_out[] = { 0x80, 0x04, 0x10, 0x02, 0x18, 0x13 };
    expect("0x8006 DESTINATION_RIGHT -> legacy DESTINATION on the right",
           arrive, sizeof arrive, (int)sizeof arrive_out, arrive_out);

    /* 0x8007 { step { distance { meters 450, display "450 m", units 1 } time 30 } dest {...} } */
    static const unsigned char pos[] = {
        0x80, 0x07,
        0x0a, 0x0e,
        0x0a, 0x0a, 0x08, 0xc2, 0x03, 0x12, 0x05, '4','5','0',' ','m',
        0x10, 0x1e,
        0x12, 0x04, 0x0a, 0x02, 0x08, 0x64,
    };
    static const unsigned char dist[] = { 0x80, 0x05, 0x08, 0xc2, 0x03, 0x10, 0x1e };
    expect("0x8007 -> 0x8005 450 m 30 s", pos, sizeof pos, (int)sizeof dist, dist);
    check("0x8007 reports destination distance separately",
          nav_translate_last()->fields_valid && nav_translate_last()->route.destination_distances == 1 &&
          nav_translate_last()->route.destination_meters == 1 && nav_translate_last()->route.eta_seconds == 0);

    static const unsigned char dest_only[] = {
        0x80, 0x07,
        0x12, 0x09, 0x0a, 0x02, 0x08, 0x00, 0x12, 0x01, 'X', 0x18, 0x00,
        0x12, 0x04, 0x0a, 0x02, 0x08, 0x0a,
    };
    expect("0x8007 with only destinations never replays the turn distance",
           dest_only, sizeof dest_only, NAV_TRANSLATE_DROP, NULL);
    check("0x8007 counts repeated destinations and considers zero valid",
          nav_translate_last()->fields_valid && nav_translate_last()->route.destination_distances == 2 &&
          nav_translate_last()->route.destination_meters == 2 && nav_translate_last()->route.eta_texts == 1 &&
          nav_translate_last()->route.eta_seconds == 1);

    /* meters -1 (unknown): keep the last good distance */
    static const unsigned char unknown[] = {
        0x80, 0x07, 0x0a, 0x0f,
        0x0a, 0x0b, 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01,
        0x10, 0x1e,
    };
    expect("0x8007 unknown distance keeps the last one", unknown, sizeof unknown, (int)sizeof dist, dist);

    /* no time in the message (Waze): not the last app's 30 s */
    static const unsigned char notime[] = { 0x80, 0x07, 0x0a, 0x05, 0x0a, 0x03, 0x08, 0xc2, 0x03 };
    static const unsigned char dist0[] = { 0x80, 0x05, 0x08, 0xc2, 0x03, 0x10, 0x00 };
    expect("0x8007 without a time drops the old time", notime, sizeof notime, (int)sizeof dist0, dist0);

    static const unsigned char reroute[] = { 0x80, 0x03, 0x08, 0x03 };
    static const unsigned char active[] = { 0x80, 0x03, 0x08, 0x01 };
    expect("0x8003 rerouting -> active", reroute, sizeof reroute, (int)sizeof active, active);
    expect("reroute clears the previous countdown", unknown, sizeof unknown, NAV_TRANSLATE_DROP, NULL);
    static const unsigned char inactive[] = { 0x80, 0x03, 0x08, 0x02 };
    expect("0x8007 fills the countdown after rerouting", pos, sizeof pos, (int)sizeof dist, dist);
    expect("0x8003 inactive passes", inactive, sizeof inactive, NAV_TRANSLATE_PASS, NULL);
    expect("inactive clears the previous countdown", unknown, sizeof unknown, NAV_TRANSLATE_DROP, NULL);
    static const unsigned char time_only[] = { 0x80, 0x07, 0x0a, 0x02, 0x10, 0x0a };
    expect("time alone cannot create a zero-meter turn", time_only, sizeof time_only, NAV_TRANSLATE_DROP, NULL);
    expect("0x8007 fills the countdown before reconnect", pos, sizeof pos, (int)sizeof dist, dist);
    nav_translate_reset();
    expect("session reset clears the previous countdown", unknown, sizeof unknown, NAV_TRANSLATE_DROP, NULL);
    static const unsigned char wide_meters[] = {
        0x80, 0x07, 0x0a, 0x08, 0x0a, 0x06, 0x08, 0x80, 0x80, 0x80, 0x80, 0x10,
    };
    expect("2^32 meters cannot wrap to a zero-meter turn",
           wide_meters, sizeof wide_meters, NAV_TRANSLATE_DROP, NULL);

    static const unsigned char other[] = { 0x80, 0x04, 0x10, 0x01 };
    expect("0x8004 passes untouched", other, sizeof other, NAV_TRANSLATE_PASS, NULL);
    expect("0x8005 passes untouched", dist, sizeof dist, NAV_TRANSLATE_PASS, NULL);

    /* kombibridge's golden frames (Desktop Head Unit captures, placeholder names) */
    static const unsigned char g1[] = { 0x80, 0x06, 0x0a, 0x6b, 0x0a, 0x18, 0x08, 0x08, 0x10, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x18, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x22, 0x2f, 0x0a, 0x2d, 0x55, 0x73, 0x65, 0x20, 0x74, 0x68, 0x65, 0x20, 0x32, 0x6e, 0x64, 0x20, 0x66, 0x72, 0x6f, 0x6d, 0x20, 0x74, 0x68, 0x65, 0x20, 0x72, 0x69, 0x67, 0x68, 0x74, 0x20, 0x6c, 0x61, 0x6e, 0x65, 0x20, 0x74, 0x6f, 0x20, 0x74, 0x75, 0x72, 0x6e, 0x20, 0x72, 0x69, 0x67, 0x68, 0x74, 0x1a, 0x04, 0x0a, 0x02, 0x08, 0x01, 0x1a, 0x04, 0x0a, 0x02, 0x08, 0x01, 0x1a, 0x04, 0x0a, 0x02, 0x08, 0x01, 0x1a, 0x06, 0x0a, 0x04, 0x08, 0x03, 0x10, 0x01, 0x1a, 0x04, 0x0a, 0x02, 0x08, 0x03 };
    static const unsigned char g1_out[] = { 0x80, 0x04, 0x0a, 0x2d, 0x55, 0x73, 0x65, 0x20, 0x74, 0x68, 0x65, 0x20, 0x32, 0x6e, 0x64, 0x20, 0x66, 0x72, 0x6f, 0x6d, 0x20, 0x74, 0x68, 0x65, 0x20, 0x72, 0x69, 0x67, 0x68, 0x74, 0x20, 0x6c, 0x61, 0x6e, 0x65, 0x20, 0x74, 0x6f, 0x20, 0x74, 0x75, 0x72, 0x6e, 0x20, 0x72, 0x69, 0x67, 0x68, 0x74, 0x10, 0x02, 0x18, 0x04 };
    expect("golden 5-lane junction -> 0x8004", g1, sizeof g1, (int)sizeof g1_out, g1_out);
    check("golden frame reports five lanes with one recommendation",
          nav_translate_last()->fields_valid && nav_translate_last()->route.lanes == 5 &&
          nav_translate_last()->route.recommended_lanes == 1);
    static const unsigned char g2[] = { 0x80, 0x06, 0x0a, 0x3b, 0x0a, 0x07, 0x08, 0x23, 0x10, 0x02, 0x18, 0x8e, 0x02, 0x22, 0x1c, 0x0a, 0x1a, 0x45, 0x78, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x20, 0x52, 0x6f, 0x61, 0x64, 0x20, 0x2f, 0x20, 0x53, 0x65, 0x63, 0x6f, 0x6e, 0x64, 0x20, 0x52, 0x69, 0x6e, 0x67, 0x1a, 0x06, 0x0a, 0x04, 0x08, 0x04, 0x10, 0x01, 0x1a, 0x04, 0x0a, 0x02, 0x08, 0x04, 0x1a, 0x04, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x09, 0x0a, 0x07, 0x08, 0x23, 0x10, 0x02, 0x18, 0x8e, 0x02 };
    static const unsigned char g2_out[] = { 0x80, 0x04, 0x0a, 0x1a, 0x45, 0x78, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x20, 0x52, 0x6f, 0x61, 0x64, 0x20, 0x2f, 0x20, 0x53, 0x65, 0x63, 0x6f, 0x6e, 0x64, 0x20, 0x52, 0x69, 0x6e, 0x67, 0x10, 0x02, 0x18, 0x0d, 0x28, 0x02, 0x30, 0x8e, 0x02 };
    expect("golden roundabout exit 2 at 270 -> 0x8004", g2, sizeof g2, (int)sizeof g2_out, g2_out);
    check("only the first step contributes lanes",
          nav_translate_last()->route.steps == 2 && nav_translate_last()->route.lanes == 3);

    static const unsigned char two_directions[] = {
        0x80, 0x06, 0x0a, 0x0e, 0x0a, 0x02, 0x08, 0x2a,
        0x1a, 0x08, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x02, 0x10, 0x01,
    };
    expect("lane data leaves the arrival pictogram intact",
           two_directions, sizeof two_directions, (int)sizeof arrive_out, arrive_out);
    check("a recommended direction without a shape does not count",
          nav_translate_last()->route.lanes == 1 && nav_translate_last()->route.recommended_lanes == 0);

    /* truncated and garbage input must not crash or overrun */
    expect("truncated 0x8006", state, 9, NAV_TRANSLATE_DROP, NULL);
    static const unsigned char bad_position[] = {
        0x80, 0x07, 0x0a, 0x04, 0x0a, 0x02, 0x08, 0x2a, 0x12, 0x03, 0x0a, 0x02, 0x08,
    };
    expect("truncated destination entry marks the position invalid",
           bad_position, sizeof bad_position, NAV_TRANSLATE_DROP, NULL);
    check("malformed position is never reported as an empty snapshot",
          !nav_translate_last()->fields_valid);
    expect("malformed position cannot prime the countdown",
           unknown, sizeof unknown, NAV_TRANSLATE_DROP, NULL);
    {
        unsigned char junk[64];
        unsigned i, seed = 1;
        unsigned char out[64];
        for (i = 0; i < 20000; ++i) {
            unsigned j, len = 2 + (seed >> 8) % 60;
            for (j = 0; j < sizeof junk; ++j) { seed = seed * 1103515245u + 12345u; junk[j] = (unsigned char)(seed >> 16); }
            junk[0] = 0x80; junk[1] = (unsigned char)(3 + i % 5);
            if (nav_translate(junk, len, out, sizeof out) > (int)len) { printf("FAIL grew\n"); ++failures; break; }
        }
        printf("PASS 20000 random messages\n");
    }
    printf("%s\n", failures ? "FAILED" : "all passed");
    return failures != 0;
}
