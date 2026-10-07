/**
 * The appearance setting: Auto follows the system, Light and Dark pin it. Pure, so the header menu,
 * the sign-in screen and the pre-paint script in index.html agree on what a stored value means.
 */

export const THEME_KEY = "sat1.theme";
export const THEME_PREFS = ["auto", "light", "dark"];

/** --surface per theme: the browser chrome colour before the orb's tint (chromeColor). */
export const THEME_COLOR = { dark: "#202027", light: "#fbfbfd" };

/** The share of the orb's colour in the header and what floats over the page (tokens.css --surface-orb). */
export const ORB_TINT = 0.07;

/** The browser chrome colour matched to the tinted header: THEME_COLOR mixed with the orb's [r, g, b]
 *  as color-mix(in srgb) does, or untinted when the orb's colour did not parse. */
export function chromeColor(theme, orbRgb) {
  const base = THEME_COLOR[theme];
  if (!orbRgb) return base;
  const n = parseInt(base.slice(1), 16);
  const s = [(n >> 16) & 255, (n >> 8) & 255, n & 255];
  return "#" + s.map((v, i) => Math.round(orbRgb[i] * ORB_TINT + v * (1 - ORB_TINT)).toString(16).padStart(2, "0")).join("");
}

/** A stored value as a preference. Nothing stored, or anything unknown, is Auto. */
export const parseThemePref = (raw) => (raw === "light" || raw === "dark" ? raw : "auto");

/** The theme a preference shows, given whether the system is in dark mode. */
export const resolveTheme = (pref, systemDark) => (pref === "auto" ? (systemDark ? "dark" : "light") : pref);
