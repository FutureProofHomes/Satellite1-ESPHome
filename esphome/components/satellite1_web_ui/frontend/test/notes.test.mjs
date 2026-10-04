/**
 * The release notes rebuild. Node has no DOMParser, so these hand notesFrom a minimal stand-in for
 * the parsed tree, shaped like GitHub's body_html for the releases already published, and read the
 * result back as markup.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { notesFrom } from "../src/lib/notes.js";

const text = (s) => ({ nodeType: 3, nodeValue: s, textContent: s });
const el = (name, attrs = {}, ...kids) => ({
  nodeType: 1,
  nodeName: name,
  childNodes: kids.map((k) => (typeof k === "string" ? text(k) : k)),
  getAttribute: (k) => attrs[k] ?? null,
  get textContent() {
    return this.childNodes.map((c) => c.textContent).join("");
  },
});
const body = (...kids) => el("BODY", {}, ...kids);

const markup = (v) => {
  if (v == null || typeof v === "boolean") return "";
  if (Array.isArray(v)) return v.map(markup).join("");
  if (typeof v === "string") return v;
  const { children, ...props } = v.props;
  const attrs = Object.entries(props)
    .filter(([, x]) => x != null)
    .map(([k, x]) => ` ${k}="${x}"`)
    .join("");
  return `<${v.type}${attrs}>${markup(children)}</${v.type}>`;
};
const notes = (...kids) => markup(notesFrom(body(...kids)));

test("Build Info goes, the author's sections stay, and headings sit under the card's h2", () => {
  // v0.2.1's notes, abridged.
  const out = notes(
    el("H2", {}, "Build Info"),
    "\n",
    el("UL", {}, el("LI", {}, "ESPHome Version: 2026.7.3"), el("LI", {}, "Commit: ", el("CODE", {}, "9a58961"))),
    "\n",
    el("H2", {}, "Summary"),
    el("P", {}, "v0.2.1 updates Satellite1 to ESPHome 2026.7.3."),
    el("H2", {}, "PRs"),
    el(
      "UL",
      {},
      el(
        "LI",
        {},
        "Use native components (",
        el("A", { class: "commit-link", href: "https://github.com/o/r/commit/6c18ee2", "data-hovercard-url": "x" }, el("TT", {}, "6c18ee2")),
        ")",
      ),
    ),
  );
  assert.equal(
    out,
    "<h3>Summary</h3><p>v0.2.1 updates Satellite1 to ESPHome 2026.7.3.</p><h3>PRs</h3><ul><li>Use native components (" +
      '<a href="https://github.com/o/r/commit/6c18ee2" target="_blank" rel="noopener noreferrer" class="ref"><code>6c18ee2</code></a>)</li></ul>',
  );
});

test("a Build Info section mid-notes ends at the next heading of its level", () => {
  // v0.2.0 put Build Info between Dashboard Builder and Internal.
  const out = notes(
    el("H2", {}, "Fixes"),
    el("UL", {}, el("LI", {}, "LED ring dimmer")),
    el("H2", {}, "Build Info"),
    el("H3", {}, "Toolchain"),
    el("UL", {}, el("LI", {}, "ESPHome Version: 2026.4.5")),
    el("H2", {}, "Internal"),
    el("UL", {}, el("LI", {}, "ci: tag-driven releases")),
  );
  assert.equal(out, "<h3>Fixes</h3><ul><li>LED ring dimmer</li></ul><h3>Internal</h3><ul><li>ci: tag-driven releases</li></ul>");
});

test("only allow-listed tags and attributes survive; code and controls are dropped whole", () => {
  const out = notes(
    el("SCRIPT", {}, "alert(1)"),
    el("STYLE", {}, "body{display:none}"),
    el("svg", {}, el("text", {}, "drawn")),
    el("P", { style: "color:red", onclick: "x()" }, "Plain ", el("A", { href: "javascript:alert(1)" }, "click"), " and ", el("A", { href: "mailto:hi@example.com" }, "mail")),
    el("DIV", { class: "snippet-clipboard-content" }, el("PRE", { class: "notranslate" }, el("CODE", {}, "1. Back up\n2. Update\n"))),
    el("P", {}, el("G-EMOJI", { alias: "warning" }, "\u26a0\ufe0f"), " Heads up", el("BR")),
    el("IMG", { src: "http://example.com/a.png", alt: "plain http" }),
    el("IMG", { src: "https://example.com/b.png", alt: "shot", width: "50%", style: "x" }),
    el("IMG", { src: "https://example.com/c.png", width: "1;x" }),
    el("BUTTON", {}, "Press"),
    el("H4", {}, "Small"),
  );
  assert.equal(
    out,
    '<p>Plain click and <a href="mailto:hi@example.com" target="_blank" rel="noopener noreferrer">mail</a></p>' +
      "<pre><code>1. Back up\n2. Update\n</code></pre>" +
      "<p>\u26a0\ufe0f Heads up<br></br></p>" +
      '<img src="https://example.com/b.png" alt="shot" width="50%" loading="lazy" referrerpolicy="no-referrer"></img>' +
      '<img src="https://example.com/c.png" alt="" loading="lazy" referrerpolicy="no-referrer"></img>' +
      "<h5>Small</h5>",
  );
});
