/**
 * The appearance setting: Auto follows the system, Light and Dark pin it. Pure, so the header menu,
 * the sign-in screen and the pre-paint script in index.html agree on what a stored value means.
 */

export const THEME_KEY = "sat1.theme";
export const THEME_PREFS = ["auto", "light", "dark"];

/** --surface per theme: the browser chrome colour, matched to the header. */
export const THEME_COLOR = { dark: "#202027", light: "#fbfbfd" };

/** A stored value as a preference. Nothing stored, or anything unknown, is Auto. */
export const parseThemePref = (raw) => (raw === "light" || raw === "dark" ? raw : "auto");

/** The theme a preference shows, given whether the system is in dark mode. */
export const resolveTheme = (pref, systemDark) => (pref === "auto" ? (systemDark ? "dark" : "light") : pref);
