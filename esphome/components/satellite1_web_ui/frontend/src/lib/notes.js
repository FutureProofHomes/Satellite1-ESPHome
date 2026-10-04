/**
 * A GitHub release's notes as elements, for the Updates card. The source is the body_html GitHub
 * renders, not the markdown: GitHub's renderer handles every construct the notes use (and the
 * #519-style links it makes from bare PR URLs), where a parser of our own would cover a subset.
 *
 * Nothing from GitHub reaches the page as HTML. This origin holds the sign-in key, so the tree is
 * rebuilt element by element from an allow-list: known tags keep only href (http, https or
 * mailto), img src (https) and alt, and width; anything else is unwrapped to its text, and the
 * tags that are code or controls rather than content are dropped whole. DOMParser's document is
 * inert - it runs no script and fetches nothing - so parsing it is safe on its own.
 */
import { h } from "preact";

/** Headings drop two levels: the page has the h1 and the card title the h2. */
const TAGS = {
  P: "p", UL: "ul", OL: "ol", LI: "li", BLOCKQUOTE: "blockquote", PRE: "pre", HR: "hr", BR: "br",
  STRONG: "strong", B: "strong", EM: "em", I: "em", DEL: "del", S: "del", CODE: "code", TT: "code",
  KBD: "kbd", SUP: "sup", SUB: "sub",
  H1: "h3", H2: "h3", H3: "h4", H4: "h5", H5: "h5", H6: "h5",
  TABLE: "table", THEAD: "thead", TBODY: "tbody", TR: "tr", TH: "th", TD: "td",
};
const DROP = new Set([
  "SCRIPT", "STYLE", "TEMPLATE", "NOSCRIPT", "IFRAME", "OBJECT", "EMBED", "SVG", "MATH",
  "FORM", "INPUT", "BUTTON", "SELECT", "TEXTAREA", "HEAD", "TITLE", "META", "LINK",
]);

/**
 * Build Info is build_release.yaml's section, not the author's: a commit hash, and the ESPHome
 * version the card already prints beside the version number.
 */
const SKIP = /^build info$/i;

const level = (n) => (n.nodeType === 1 && /^H[1-6]$/i.test(n.nodeName) ? Number(n.nodeName[1]) : 0);

function children(list) {
  const out = [];
  let skipping = 0;
  for (const c of Array.from(list)) {
    const lvl = level(c);
    if (skipping && (!lvl || lvl > skipping)) continue;
    skipping = 0;
    if (lvl && SKIP.test((c.textContent || "").trim())) {
      skipping = lvl;
      continue;
    }
    const v = convert(c);
    if (v != null) out.push(v);
  }
  return out;
}

function convert(n) {
  if (n.nodeType === 3) return n.nodeValue;
  if (n.nodeType !== 1) return null;
  const name = n.nodeName.toUpperCase();
  if (DROP.has(name)) return null;
  if (name === "IMG") {
    const src = n.getAttribute("src") || "";
    if (!/^https:\/\//i.test(src)) return null;
    const width = n.getAttribute("width") || "";
    return h("img", {
      src,
      alt: n.getAttribute("alt") || "",
      width: /^\d+%?$/.test(width) ? width : undefined,
      loading: "lazy",
      referrerpolicy: "no-referrer",
    });
  }
  const kids = children(n.childNodes);
  if (name === "A") {
    const href = n.getAttribute("href") || "";
    if (!/^(https?:\/\/|mailto:)/i.test(href)) return kids;
    // GitHub's own references (#519, a commit, @someone) read as secondary to the line they end.
    const ref = /\b(issue-link|commit-link|user-mention)\b/.test(n.getAttribute("class") || "");
    return h("a", { href, target: "_blank", rel: "noopener noreferrer", class: ref ? "ref" : undefined }, kids);
  }
  const tag = TAGS[name];
  if (!tag) return kids;
  return tag === "br" || tag === "hr" ? h(tag, null) : h(tag, null, kids);
}

/** The elements a parsed node's children become; exported for the tests, which have no DOMParser. */
export const notesFrom = (root) => children(root.childNodes);

/** A release's body_html as elements, or null when nothing readable is left of it. */
export function releaseNotes(html) {
  if (!html) return null;
  const nodes = notesFrom(new DOMParser().parseFromString(html, "text/html").body);
  return nodes.some((n) => typeof n !== "string" || n.trim()) ? nodes : null;
}
