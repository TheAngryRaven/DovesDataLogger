#pragma once

///////////////////////////////////////////
// SETTING VALUE PARSING (pure unit)
//
// Strict numeric parsing for /SETTINGS.json values, extracted because
// atoi() is the wrong tool here and the wrongness is invisible.
//
// atoi("") and atoi("garbage") both answer 0, with no way to tell either
// from a real "0". Every numeric setting in BirdsEye.ino is read as
//
//     if (getSetting(key, buf, n)) { const int v = atoi(buf);
//                                    if (inRange(v)) applyIt(v); }
//
// which is safe only while 0 is OUT of the accepted range — true for
// rev_limit (floor 1000) and temp1_alert_c (floor 50), and FALSE for
// every setting plan 0006/0010 added. For led_brightness, 0 is a real and
// destructive value: it disables the LEDs and the 5 V boost rail never
// comes up. So a blank or corrupt led_brightness did not "clamp back to
// the compiled-in default per the house idiom" as its comment claims — it
// silently killed the strip, and looked exactly like dead hardware.
// Same shape for led_brightness_night (0 = blank strip) and the two
// day/night hours (0 = midnight, a legitimate value).
//
// parseIntSetting() answers false for anything that is not a complete
// integer, so the caller's existing range check keeps the compiled-in
// default — which is what those comments always meant.
///////////////////////////////////////////

namespace setting_parse {

/**
 * @brief Parse a settings value as a complete decimal integer.
 *
 * Accepts optional leading whitespace and an optional +/- sign, then one
 * or more digits, then optional trailing whitespace, and nothing else.
 * Rejects "" / " " / "abc" / "12abc" / "1.5" / "0x10" — anything atoi()
 * would quietly turn into a number.
 *
 * @param s    NUL-terminated value (may be nullptr).
 * @param out  Receives the value only when this returns true.
 * @return true if the whole string is an integer that fits an int.
 */
bool parseIntSetting(const char* s, int* out);

}  // namespace setting_parse
