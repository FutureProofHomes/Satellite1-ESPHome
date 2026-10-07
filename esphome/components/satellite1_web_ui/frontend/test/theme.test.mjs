/**
 * The appearance setting and the orb colour's theme tokens: what a stored preference means, which
 * theme it shows, and that every orb colour yields text and fills that pass contrast in both themes.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { colorRgb, contrast, INK_ON, orbTokens } from "../src/lib/orb.js";
import { valueAt } from "../src/lib/range.js";
import { chromeColor, parseThemePref, resolveTheme, THEME_COLOR } from "../src/lib/theme.js";

test("nothing stored, or anything unknown, is Auto", () => {
  assert.equal(parseThemePref(null), "auto");
  assert.equal(parseThemePref(""), "auto");
  assert.equal(parseThemePref("sepia"), "auto");
  assert.equal(parseThemePref("auto"), "auto");
  assert.equal(parseThemePref("light"), "light");
  assert.equal(parseThemePref("dark"), "dark");
});

test("Auto follows the system; Light and Dark ignore it", () => {
  assert.equal(resolveTheme("auto", true), "dark");
  assert.equal(resolveTheme("auto", false), "light");
  assert.equal(resolveTheme("light", true), "light");
  assert.equal(resolveTheme("dark", false), "dark");
});

test("the browser chrome takes the header's tint: 7% of the orb in sRGB, as color-mix does", () => {
  // The values Chrome computes for --surface-orb with the default violet (#a78bfa).
  assert.equal(chromeColor("light", [167, 139, 250]), "#f5f3fd");
  assert.equal(chromeColor("dark", [167, 139, 250]), "#292736");
  assert.equal(chromeColor("dark", null), THEME_COLOR.dark);
});

const hex = (h) => colorRgb(h);
const WHITE = [255, 255, 255];
const PRESETS = ["#a78bfa", "#c084fc", "#38bdf8", "#34d399", "#fbbf24", "#fb7185", "#fb923c", "#60a5fa"];
const CUSTOM = [0, 60, 120, 180, 240, 300].map((h) => `hsl(${h}, 70%, 70%)`);

test("orb colours parse from hex and the custom picker's hsl", () => {
  assert.deepEqual(colorRgb("#fff"), [255, 255, 255]);
  assert.deepEqual(colorRgb("#34d399"), [52, 211, 153]);
  assert.deepEqual(colorRgb("hsl(0, 100%, 50%)"), [255, 0, 0]);
  assert.deepEqual(colorRgb("hsl(240, 100%, 50%)"), [0, 0, 255]);
  assert.equal(colorRgb("tomato"), null);
  assert.equal(colorRgb(undefined), null);
});

test("every orb colour gives a fill white text passes AA on, in both themes", () => {
  for (const c of [...PRESETS, ...CUSTOM]) {
    for (const theme of ["dark", "light"]) {
      const t = orbTokens(c, theme);
      assert.ok(contrast(hex(t.fill), WHITE) >= 4.5, `${c} ${theme} fill ${t.fill}`);
    }
  }
});

test("orb ink reads on every backdrop of the theme", () => {
  for (const c of [...PRESETS, ...CUSTOM]) {
    assert.ok(contrast(hex(orbTokens(c, "light").ink), INK_ON.light) >= 4.5, `${c} light`);
    assert.ok(contrast(hex(orbTokens(c, "dark").ink), INK_ON.dark) >= 4.5, `${c} dark`);
  }
});

test("the fill keeps the colour's hue and only darkens as far as it has to", () => {
  const t = orbTokens("#34d399", "light");
  assert.equal(t.fill, "#1d845e");
  assert.ok(contrast(hex(t.fill), WHITE) < 4.9);
  assert.equal(t.ctl, t.fill);
  assert.equal(t.ctlOn, "#fff");
  // Already dark enough: untouched.
  assert.equal(orbTokens("#2563eb", "light").fill, "#2563eb");
});

test("dark mode controls glow in the raw colour, with a dark tick on light ones", () => {
  const t = orbTokens("#fbbf24", "dark");
  assert.equal(t.ctl, "#fbbf24");
  assert.equal(t.ink, "#fbbf24");
  assert.equal(t.ctlOn, "#111113");
  assert.equal(t.a20, "rgba(251,191,36,0.2)");
});

test("an unparseable orb colour falls back to the default violet", () => {
  assert.deepEqual(orbTokens("nonsense", "dark"), orbTokens("#a78bfa", "dark"));
});

test("a touch on a slider's track maps to the value under it", () => {
  // 200px wide, 28px thumb: the thumb centre travels 14..186.
  assert.equal(valueAt(14, 0, 200, 0, 100, 1, 28), 0);
  assert.equal(valueAt(186, 0, 200, 0, 100, 1, 28), 100);
  assert.equal(valueAt(100, 0, 200, 0, 100, 1, 28), 50);
  assert.equal(valueAt(-50, 0, 200, 0, 100, 1, 28), 0);
  assert.equal(valueAt(999, 0, 200, 0, 100, 1, 28), 100);
  assert.equal(valueAt(110, 10, 200, 0, 100, 1, 28), 50);
  assert.equal(valueAt(57, 0, 200, 0, 600, 10, 28), 150);
  assert.equal(valueAt(100, 0, 200, 0, 1, 0.05, 28), 0.5);
  assert.equal(valueAt(100, 0, 200, 0, 8, 3, 28), 3);
  assert.equal(valueAt(186, 0, 200, 0, 8, 3, 28), 6);
});
