/**
 * Where the wake word pickers' lists come from: source repositories, enumerated live by the
 * browser. Nothing is copied or re-hosted (owner decision, September 2026) - these are the same
 * URLs an ESPHome build would pull from, and the two defaults ship pre-added.
 *
 * The browser only does discovery. The device downloads the manifest itself and re-validates
 * everything, so nothing here is trusted: a source that cannot be enumerated (CORS, rate limit)
 * still yields addable entries, and a pasted URL the browser cannot even fetch is handed to the
 * device anyway, whose validation ladder is the real gate.
 *
 * Enumeration, cheapest first:
 *  1. A `wake_word_manifest.json` catalog at the repo root (Tater's convention - 778 words in one
 *     fetch, no API rate limit).
 *  2. GitHub's git/trees API, one recursive request for every path in the repo, then the manifests
 *     themselves from raw.githubusercontent - but only when there are few enough to fetch politely.
 *     Past that, entries are built from filenames alone and verified by the device on add.
 */

const LS_SOURCES = "sat1.wakesources";

export const DEFAULT_SOURCES = [
  { url: "https://github.com/esphome/micro-wake-word-models", label: "ESPHome" },
  { url: "https://github.com/TaterTotterson/Tater-Wake-Words", label: "TaterTotterson" },
];

/** Where "Request one" and "train your own" point (owner's links, September 2026). The request
 *  link lands on the catalog's own README section explaining the issue automation - an issue
 *  titled `mww: <phrase>` triggers their trainer, which publishes the model and updates the
 *  catalog this file reads. The trainer link is the Docker image that automation runs. */
export const REQUEST_WORD_URL = "https://github.com/TaterTotterson/Tater-Wake-Words#request-a-wake-word";
export const TRAIN_URL = "https://github.com/TaterTotterson/microWakeWord-Trainer-Nvidia-Docker";

/** The customer's sources: the defaults plus whatever they pasted, minus whatever they removed. */
export function readSources() {
  try {
    const v = JSON.parse(localStorage.getItem(LS_SOURCES));
    if (Array.isArray(v)) return v;
  } catch {
    /* fall through to the defaults */
  }
  return DEFAULT_SOURCES;
}

export function writeSources(list) {
  try {
    localStorage.setItem(LS_SOURCES, JSON.stringify(list));
  } catch {
    /* not remembering is survivable */
  }
}

/** "user/repo" from a GitHub URL, or null for anything else. */
function ghRepo(url) {
  const m = /^https?:\/\/(?:www\.)?github\.com\/([^/]+)\/([^/#?]+)/.exec(url);
  return m ? { owner: m[1], repo: m[2].replace(/\.git$/, "") } : null;
}

const pretty = (slug) =>
  slug
    .replace(/[_-]+/g, " ")
    .replace(/\.json$/i, "")
    .replace(/\b\w/g, (c) => c.toUpperCase());

const fetchJson = async (url) => {
  const r = await fetch(url, { signal: AbortSignal.timeout(10000), cache: "no-store" });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
};

/**
 * Whether a manifest the browser managed to read is one this firmware can run: microWakeWord,
 * manifest version 2, and the 10ms feature step the audio frontend is compiled for. The firmware
 * re-checks all of it - this filter exists so the picker never offers a word that is certain to be
 * refused. The concrete case: the ESPHome repository still hosts its old version-1 manifests
 * (models/alexa.json and friends) right next to the v2 ones, trained for the pre-2024.7 framework;
 * without this the list showed two Alexas, one of which always failed.
 */
const compatible = (m) =>
  m && m.type === "micro" && m.wake_word && m.version === 2 && m.micro && (m.micro.feature_step_size ?? 10) === 10;

/**
 * Orders a source's words for the picker and marks what needs disambiguating: alphabetical by
 * phrase (the catalogs arrive grouped by training-generation folder, which scatters the alphabet),
 * and where the same phrase appears more than once - Tater's collection has 39 such labels, three
 * "Computer"s among them - each copy gets its `ver` tag shown, the training generation from its
 * folder (v1..v3 are the original seed batches, v6 the issue-trained additions). A phrase that is
 * unique keeps a clean row; the tag exists to tell twins apart, not to decorate.
 */
function finish(entries) {
  entries.sort((a, b) => a.word.localeCompare(b.word) || (a.ver || "").localeCompare(b.ver || ""));
  const seen = new Map();
  for (const e of entries) seen.set(e.word.toLowerCase(), (seen.get(e.word.toLowerCase()) || 0) + 1);
  for (const e of entries) e.dup = seen.get(e.word.toLowerCase()) > 1;
  return entries;
}

/** The training-generation tag from a catalog folder or path: "microWakeWordsV3/..." -> "v3". */
const verOf = (text) => {
  const m = /v(\d+)\//i.exec(`${text}/`);
  return m ? `v${m[1]}` : "";
};

/* One enumeration per source per page load. Keyed on the URL; a rejected promise is evicted so a
   rate-limited attempt can be retried by reopening the picker. */
const cache = new Map();

/**
 * The words a source offers: [{word, url, langs, size, exp, unverified}].
 * `url` is the manifest the device will be told to download. `exp` marks ESPHome's experiments
 * folder ("minimally tested", their words). `unverified` means the browser never read the manifest
 * - the entry is still addable, and the device checks what the browser could not.
 */
export function enumerateSource(source) {
  if (cache.has(source.url)) return cache.get(source.url);
  const p = enumerate_(source).catch((e) => {
    cache.delete(source.url);
    throw e;
  });
  cache.set(source.url, p);
  return p;
}

async function enumerate_(source) {
  const gh = ghRepo(source.url);

  // A direct manifest URL is a one-word source. CORS may block the read (raw GitHub allows it,
  // arbitrary hosts vary); an unreadable entry survives as unverified, because the device is the
  // validator - but one the browser *could* read and found incompatible is dropped here, so the
  // list never offers a word the firmware is certain to refuse (owner request, September 2026).
  if (!gh && /\.json($|\?)/.test(source.url)) {
    let m = null;
    try {
      m = await fetchJson(source.url);
    } catch {
      return [{ word: pretty(source.url.split("/").pop() || ""), url: source.url, langs: [], size: 0, unverified: true }];
    }
    if (compatible(m)) {
      return [{ word: m.wake_word, url: source.url, langs: m.trained_languages || [], size: 0 }];
    }
    throw new Error("not a compatible microWakeWord manifest");
  }
  if (!gh) throw new Error("Not a GitHub repository or a model .json link");

  const raw = (branch, path) => `https://raw.githubusercontent.com/${gh.owner}/${gh.repo}/${branch}/${path}`;

  // 1. The catalog convention: everything in one fetch, no rate limit.
  for (const branch of ["main", "master"]) {
    try {
      const cat = await fetchJson(raw(branch, "wake_word_manifest.json"));
      const entries = Array.isArray(cat) ? cat : cat.entries;
      if (Array.isArray(entries) && entries.length) {
        return finish(
          entries
            // A catalog entry that declares its manifest version declares it for this filter; one
            // that does not is left in, and the device's own validation is the gate.
            .filter((e) => e.version == null || e.version === 2)
            .map((e) => ({
              word: e.label || e.name || pretty(e.slug || ""),
              url: e.url || e.download_url || (e.path ? raw(branch, e.path) : null),
              langs: e.trained_languages || [],
              size: e.size || 0,
              ver: verOf(e.source || e.path || ""),
            }))
            .filter((e) => e.word && e.url)
        );
      }
    } catch {
      /* no catalog on this branch; keep going */
    }
  }

  // 2. The trees API: one request lists every path (and blob size) in the repo.
  const meta = await fetchJson(`https://api.github.com/repos/${gh.owner}/${gh.repo}`);
  const branch = meta.default_branch || "main";
  const tree = await fetchJson(`https://api.github.com/repos/${gh.owner}/${gh.repo}/git/trees/${branch}?recursive=1`);
  const paths = (tree.tree || []).filter((t) => t.type === "blob");
  const sizes = new Map(paths.map((t) => [t.path, t.size || 0]));
  const manifests = paths
    .map((t) => t.path)
    .filter(
      (p) =>
        p.endsWith(".json") &&
        !/(^|\/)(wake_word_manifest|package(-lock)?|tsconfig|manifest)\.json$/.test(p) &&
        !/(^|\/)vad\.json$/.test(p) &&
        !p.endsWith(".esphome.json") &&
        // Excluded outright, not offered behind a caveat (owner decision, September 2026): the
        // folder's own README says "minimally trained and tested, not supported in any way", and
        // hardware time proved it - a word that never fires reads as our bug, not theirs.
        !p.includes("/experiments/")
    );
  if (!manifests.length) throw new Error("No wake word models found in this repository");

  // Few enough to read politely: fetch each manifest for the real phrase and languages, and take
  // the model's size off the tree. Beyond that, filenames stand in and the device verifies on add.
  if (manifests.length <= 40) {
    const out = await Promise.all(
      manifests.map(async (p) => {
        try {
          const m = await fetchJson(raw(branch, p));
          if (!compatible(m)) return null;
          const dir = p.slice(0, p.lastIndexOf("/") + 1);
          const model = String(m.model || "").replace(/^\.\//, "");
          return {
            word: m.wake_word,
            url: raw(branch, p),
            langs: m.trained_languages || [],
            size: sizes.get(/^https?:/.test(model) ? "" : dir + model) || 0,
            ver: verOf(p),
          };
        } catch {
          return null;
        }
      })
    );
    const found = out.filter(Boolean);
    if (!found.length) throw new Error("No wake word models found in this repository");
    return finish(found);
  }

  return finish(
    manifests.map((p) => ({
      word: pretty(p.split("/").pop()),
      url: raw(branch, p),
      langs: [],
      size: 0,
      ver: verOf(p),
      unverified: true,
    }))
  );
}

/** Speaks the phrase with the browser's own voices - zero firmware bytes. Callers hide the glyph
 *  when this reports unavailable, so no browser shows a dead button. */
export const canSpeak = () => typeof speechSynthesis !== "undefined";
export function speak(word) {
  try {
    speechSynthesis.cancel();
    speechSynthesis.speak(new SpeechSynthesisUtterance(word));
  } catch {
    /* a browser that lies about support loses the preview, nothing else */
  }
}
