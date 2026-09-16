/**
 * Builds the on-device web app into one self-contained document at ../dist/index.html.
 *
 * Three decisions are baked in here rather than left to taste:
 *
 * One document, not index.html + app.js + app.css. Every extra file is another request on a
 * server whose hard ceiling is 7 sockets, and it buys no cache granularity: the whole bundle
 * changes with every firmware update anyway.
 *
 * dist/ holds the *uncompressed* HTML. Codegen does the gzipping, the same way
 * satellite1_radar/__init__.py already does for the tuner pages. Committing the .gz instead
 * would make the CI drift check flaky, because gzip streams are not byte-reproducible across
 * zlib versions, and it would turn a reviewable diff into a binary blob.
 *
 * package.json pins esbuild and preact to exact versions, no carets. The CI job asserts the
 * committed bundle is byte-identical to a fresh build, and esbuild's minifier output moves
 * between releases - a caret would make that check fail on a dependency bump nobody made.
 */
import * as esbuild from "esbuild";
import { gzipSync } from "node:zlib";
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const dist = join(here, "..", "dist");

const shared = {
  bundle: true,
  minify: true,
  target: "es2020",
  legalComments: "none",
  write: false,
  absWorkingDir: here,
};

const js = await esbuild.build({
  ...shared,
  entryPoints: ["src/main.jsx"],
  format: "iife",
  jsx: "automatic",
  jsxImportSource: "preact",
});

const css = await esbuild.build({
  ...shared,
  entryPoints: ["src/app.css"],
});

// HTML comments are stripped before the CSS and JS go in, and not after: minified JS can legitimately
// contain "<!--" inside a string literal, and a regex run over the assembled document could eat it.
// esbuild already drops comments from both other languages, so without this the only developer prose
// that reaches flash would be whatever is in index.html - which is where the least obvious code lives.
const html = readFileSync(join(here, "src", "index.html"), "utf8")
  .replace(/\n?[ \t]*<!--[\s\S]*?-->/g, "")
  .replace("/*%CSS%*/", () => css.outputFiles[0].text.trimEnd())
  .replace("/*%JS%*/", () => js.outputFiles[0].text.trimEnd());

if (html.includes("%CSS%") || html.includes("%JS%")) {
  throw new Error("src/index.html is missing a /*%CSS%*/ or /*%JS%*/ placeholder");
}

mkdirSync(dist, { recursive: true });
writeFileSync(join(dist, "index.html"), html);

// Close to, but not exactly, the number the firmware embeds. Node's zlib and CPython's zlib
// disagree by a few tens of bytes at the same level on the same input, so treat this as accurate
// to within about half a percent. The authoritative figure is the one __init__.py logs at compile
// time, since those are the bytes that go into the image.
// 48KB, raised from 40KB in September 2026 when the media footer landed at 99.8% of the old line.
// The number is a self-imposed discipline, not a hardware limit - the bundle lives in flash and is
// served from PSRAM either way - so the raise bought room for the visual-polish pass without
// changing what the device pays: about 8KB more flash and the same PSRAM copy at boot.
const gz = gzipSync(html, { level: 9 }).length;
const pct = ((gz / 49152) * 100).toFixed(0);
console.log(`dist/index.html  ${html.length} B raw  ${gz} B gzipped  (${pct}% of the 48KB target)`);
