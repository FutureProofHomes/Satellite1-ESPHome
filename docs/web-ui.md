# The on-device web app

Satellite1 serves its own configuration app from the device, at `http://<device>/`. It needs no
internet connection, no CDN and no Home Assistant to load — everything it needs is in flash.

This note covers how it is put together and what it costs. For the wording of every control and
tooltip, see [web-ui-copy.md](web-ui-copy.md); for TTS routing and ducking specifically, including the
breaking changes in the current release, see [TTS-Routing.md](TTS-Routing.md).

## What it is

A Preact single-page app, built with esbuild into one self-contained HTML document — markup, styles and
JavaScript in one file — which is gzipped at compile time and embedded in the firmware as a
`progmem_array`. There is no filesystem, no SPIFFS image and no second artifact to keep in step.

Four routes, all on the hash:

| Route | What it does |
|---|---|
| `#/controls` | Sensor readings with tap-to-calibrate, voice, timers, transcript, speaker, LED ring |
| `#/presence` | Live radar plot or gate energies, and the radar's own settings |
| `#/config` | TTS routing and area ducking, as a tree over your Home Assistant areas |
| `#/diagnostics` | Memory, firmware, buttons, live log, and maintenance actions |

Routing is on the hash rather than on the path because ESPHome's HTTP server has a single wildcard
handler per method and no notion of client-side routes. A path-based router would 404 on reload for
every route except `/`. The hash never reaches the server, which also means a bookmarked route survives
a firmware update.

## Who owns `/`

Both `web_server` and this component want the root URL. The winner is decided by setup priority:
`satellite1_web_ui` registers at `setup_priority::WIFI` (250) and `web_server` at `WIFI - 1.0f` (249),
so ours registers first and claims `/`. ESPHome's own dashboard is still reachable — it simply no longer
answers the root. The app is also mounted at `/ui/`.

Everything else routes normally, which is what makes the app small: it does not reimplement entity
control. Reads come from `web_server`'s `/events` stream, writes go to its REST entity endpoints, and
only the handful of things `web_server` has no concept of get custom endpoints.

| Endpoint | Purpose |
|---|---|
| `GET /api/sat1/state` | Device facts and the entity-name table |
| `GET /api/sat1/voice` | Assistant phase, timers, transcript ring |
| `GET /api/sat1/ha` | The cached Home Assistant area and player tree |
| `GET`/`POST /api/sat1/sel` | The routing and ducking selection |
| `/api/v1/...` | The radar's own API, served by `satellite1_radar` |

Heap, PSRAM, loop time, uptime and reset reason are read straight from the IDF inside the handler rather
than mirrored into entities, so the Diagnostics page costs no entity overhead. The one exception is chip
temperature, which needs the temperature-sensor driver and therefore is a real sensor.

## Authentication

`web_server` runs with auth enabled, so every request — the app, the entity API and the radar API — is
behind Basic auth. The username is set in YAML because it must be non-empty before any handler
registers. The password defaults to a value derived from the device MAC, can be overridden with a
`web_ui_password` substitution, and is published as a diagnostic text sensor because that is the only
way a customer can discover the generated value. Anyone who can read that sensor can already control the
device through Home Assistant, so it gives away nothing they did not have.

## The bundle

Measured, from the build that produced this note:

| | Bytes |
|---|---|
| Raw HTML document | 67,615 |
| Gzipped, as embedded | **23,231** |
| Budget | 40,960 |

That is 57% of the ceiling. The gzip figure is the one that matters, since that is what occupies flash
and what crosses the network. It is produced by `gzip.compress(html, compresslevel=9)` in the
component's codegen, so the number reported at compile time is the number that ships.

`dist/index.html` is committed, and CI rebuilds it and fails on `git diff --exit-code` against `dist`,
so the committed bundle cannot drift from the source it was built from. To rebuild locally:

```
cd esphome/components/satellite1_web_ui/frontend
npm install
npm run build
```

The bundle is served with `Content-Encoding: gzip`, `Cache-Control: no-cache` and an ETag. `no-cache`
does not mean "do not cache" — it means "revalidate", so a loaded tab costs one conditional request that
answers 304, while a firmware update is picked up immediately. Caching the bundle for a fixed period
would mean a stale app after an update, which is a bug rather than an optimisation.

## Sockets and the event stream

Neither `satellite1_web_ui` nor `satellite1_radar` calls `consume_sockets()`, and both say so in a
comment so nobody adds one back. Raising it inflates `CONFIG_LWIP_MAX_SOCKETS` without lifting
`esp_http_server`'s own `max_open_sockets = 7`, which is the number that actually bounds concurrent
tabs. Neither component adds a listener: both are handlers on the socket `web_server_base` already
reserves.

Phase 1 removed a second server rather than adding one. The radar tuner used to run its own
`esp_http_server` and reserve a socket for it; it is now a handler on the shared server.

The budget the app is written to is **one SSE connection per browser tab, and fetches serialised behind
it**. Writes go through a single-flight queue so a burst of taps cannot open a socket per tap. The one
route that polls is Presence, and only while it is open: the live radar read is chained on completion
rather than run on an interval, so a slow device stretches the gap instead of queueing requests.

## What it costs

Both figures are from clean compiles of `config/satellite1.yaml`, before and after all of this work.
Before is commit `7c181b7`, the last commit with no web app; after is the current tree.

| | Before | After | Delta |
|---|---|---|---|
| Static RAM | 121,975 | 123,731 | **+1,756** |
| Flash | 2,722,459 | 2,821,471 | **+99,012** |

So the whole thing — the SPA, the component, the Home Assistant data layer, the selection store, the
Presence route, and moving the radar tuner onto the shared server — costs about 97 KB of flash and 1.7 KB
of static RAM. Flash is 34.7% used of 8,126,464.

Runtime, measured on hardware over HTTP after 25 minutes of uptime:

| | Free | Of |
|---|---|---|
| Internal heap | 148,600 | 299,812 |
| PSRAM | 4,493,284 | 5,579,868 |

Largest free internal block was 131,072 bytes and the longest single loop pass 24 ms. Internal heap is
the number that matters, because it is what audio buffers and the network stack allocate from; PSRAM is
plentiful by comparison, and cached Home Assistant payloads are deliberately put there.

Boot time has not yet been measured as a before-and-after pair. Doing it honestly needs the baseline
firmware flashed to the same hardware, which has not been done.

## Why the browser never calls Home Assistant

It cannot. A browser on your phone has no credential for Home Assistant, and asking people to paste a
long-lived token into a device page would be both a poor experience and a genuine security problem. So
every piece of Home Assistant data the app shows is fetched by the *device*, over the native API
connection it already has, and cached.

That is what `GET /api/sat1/ha` serves: one shaped payload listing your areas, the media players in
each, and which of them are Satellite1s. It is refreshed on triggers rather than polled, because it
describes a house, which changes rarely. The payload is capped and the app is told when it was
truncated, so a very large installation degrades visibly rather than silently.

The consequence worth knowing is that when Home Assistant is unreachable, the app keeps working for
everything local — sensors, LEDs, the radar, the log, maintenance — and greys out only what genuinely
depends on Home Assistant, with a one-line explanation of the problem and the fix.

## The routing and ducking selection

The Config route's two trees write one selection that lives on the device, in NVS, and is read and
written at `GET`/`POST /api/sat1/sel`. It stores whole areas by id, individually chosen players, and
carve-outs from a whole area — so "the whole living room except the TV" is stored as an area plus one
exclusion, and a speaker added to that area later is picked up without anyone revisiting the setting.

The two switches Home Assistant keeps are projections of that selection rather than peers of it, so
there is nothing to keep in step. Both carry `restore_mode: DISABLED`, without which ESPHome acts on a
restored value at boot and the turn action writes a stale state into the store.

This replaced a comma-separated text entity, and it is a breaking change with renamed and deleted
entities. [TTS-Routing.md](TTS-Routing.md) has the upgrade notes.

## Rebuilding and conventions

All user-facing strings live in `frontend/src/copy.js` and are mirrored into
[web-ui-copy.md](web-ui-copy.md), so wording reviews as one diff rather than being hunted through JSX.
Both build paths must keep working: `esphome compile config/satellite1.yaml` locally, and the ESPHome
Device Builder through `config/satellite1.dashboard.yaml`.
