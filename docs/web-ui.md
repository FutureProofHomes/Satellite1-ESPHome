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

Five routes, all on the hash, in this nav order:

| Route | What it does |
|---|---|
| `#/home` | Sensor readings with tap-to-calibrate, media, the Assistant card (phase, transcript, mute, assistant volume), timers, LED ring |
| `#/wake-word` | Which wake words the device answers to, sensitivity, wake chime, and the stop word |
| `#/audio` | TTS routing and area ducking as a tree over your Home Assistant areas, then the speaker's own wiring |
| `#/presence` | Live radar plot or gate energies, and the radar's own settings |
| `#/diagnostics` | Memory, firmware, live logs, and maintenance actions |

`#/home` and `#/audio` were `#/controls` and `#/config` until the September 2026 rename pass, which
also gave the wake words card its own `#/wake-word` route; the old hashes still resolve to the new
routes so bookmarks keep working.

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

Every endpoint above builds its body in an `AsyncResponseStream` — the httpd task stack is 4,352 bytes,
so a response cannot be assembled on it — except `GET /api/sat1/ha`, which is chunked straight out of
PSRAM. A stream accumulates into a `std::string`, and with `CONFIG_SPIRAM_USE_CAPS_ALLOC` that is the
internal heap, so printing the cached payload copied up to 24 KB of PSRAM into the scarcest memory on the
device, needing a contiguous block of roughly twice the payload while the string grew. Exceptions are
compiled out, so the failed allocation aborted the chip rather than failing the request: on a WiFi build
the app rebooted the device every time it asked for this data. The handler now writes the header, the
payload pointer and the closing brace as three `httpd_resp_send_chunk` calls, so the payload never
touches the internal heap and the response carries no `Content-Length`, which is what chunked means.

The write side of that payload had the same bug at the other end. The capture lambda in
`common/web_ui_ha.yaml` used to `serializeJson` into a `std::string`, which is the same several kilobytes
on the same heap, and it aborted the device on the sync itself once the read path stopped doing it first.
So the component hands out its PSRAM buffer instead: `stage_ha_payload(capacity)` returns somewhere to
write, the lambda serialises ArduinoJson's output directly into it, and `commit_ha_payload(len, rung)`
publishes it by swapping that buffer with the one being served. The two buffers alternate, so after the
first two syncs a resync allocates nothing and copies nothing, and the payload's bytes go from the API
message to PSRAM and nowhere else.

The routes that do still stream are guarded. If the largest free internal block is under 6 KB when one of
them arrives, it is answered `503` with a `Retry-After` rather than attempted — a stale reading in the app
instead of a reboot. The frontend already treats a failed fetch as "keep what I have", so nothing there had
to change. That threshold is a floor and not a comfort margin, which matters more than it sounds: the first
version asked for 16 KB, which is more contiguous internal memory than a WiFi build of this firmware ever
has — measured at 7,680 bytes largest free block on hardware — so it refused every streamed route and left
the app with no data at all. `GET /api/sat1/state` is the largest of them at about 2 KB, peaking at a 4 KB
allocation while the 2 KB buffer it is copying from is still alive, so 6 KB clears a real response and only
speaks when one genuinely could not have been served.

## Authentication

`web_server` runs with auth enabled, so every request — the app, the entity API and the radar API — is
behind Basic auth. The username is set in YAML because it must be non-empty before any handler
registers. The password defaults to a value derived from the device MAC, can be overridden with a
`web_ui_password` substitution, and is published as a diagnostic text sensor because that is the only
way a customer can discover the generated value. Anyone who can read that sensor can already control the
device through Home Assistant, so it gives away nothing they did not have.

## The bundle

Measured, from the build that produced this note (September 2026, after the full visual-polish
pass: motion language, the redesigned nav drawer, the radar hero treatment, media finishing, the
type ramp, empty states, the brand gradient (drawer indicator and slider fills only - never on
buttons, by the owner's call), header blur, tree chips, first-load shimmers and the
desktop two-column layout - all eleven rounds plus the request-queue deadline, for 1,880 B over the
pre-polish bundle):

| | Bytes |
|---|---|
| Raw HTML document | 129,095 |
| Gzipped, as embedded | **42,775** |
| Budget | 49,152 |

That is 87% of the ceiling. The budget was 40,960 B until September 2026, when the media footer
landed at 99.8% of it; the owner raised the line to 48KB for the visual-polish pass. The number is
a self-imposed discipline rather than a hardware limit - flash sits under 43% used either way - so
the raise cost about 8KB of flash headroom and nothing else, and the discipline continues against
the new line. The gzip figure is the one that matters, since that is what occupies flash
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

Runtime, measured on hardware over HTTP after 25 minutes of uptime, on an **ethernet** build:

| | Free | Of |
|---|---|---|
| Internal heap | 148,600 | 299,812 |
| PSRAM | 4,493,284 | 5,579,868 |

Largest free internal block was 131,072 bytes and the longest single loop pass 24 ms. Internal heap is
the number that matters, because it is what audio buffers and the network stack allocate from; PSRAM is
plentiful by comparison, and cached Home Assistant payloads are deliberately put there.

The media footer work (September 2026) was measured on a WiFi dev unit rather than the ethernet
device above, so the numbers read against the WiFi paragraph below rather than the table: with the
firmware playing a group stream, serving the app, and the Music Assistant relay having synced, free
internal heap settled at 52,336 with a 30,720 largest block and PSRAM 3,438,748 free — no worse than
the same device before the feature (48,752 free measured minutes earlier on the previous firmware),
and stable through a hammer of the new endpoints.
Static RAM moved from 42.9% to 42.9% (+8 bytes) across the whole feature, because every buffer it
adds — the metadata fragment, the members payload, its staging twin — is PSRAM, allocated on first
use.

A WiFi build of the same tree has far less of it — tens of KB free rather than 148 KB, and a smaller total
— and the difference is not this component. ESPHome's `wifi` component applies a high-performance profile
whenever any component asks for one, and both `speaker.media_player` and `sendspin` do: the build gets
`CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=16` and `CONFIG_ESP_WIFI_STATIC_TX_BUFFER_NUM=8`, roughly 38 KB of
DMA-capable internal buffers, and only WiFi builds pull in the BLE stack through `common/wifi_improv.yaml`
for Improv provisioning. So the numbers above are the ethernet picture, not a regression when a WiFi
device reads lower. Anything on the device that allocates a few KB of internal RAM is closer to the edge
on WiFi, which is why the two measures above are worth reading as a pair rather than as one number.

Note that plain `sdkconfig_options:` in YAML cannot be used to trim those WiFi buffers: the `wifi`
component writes them from its own `to_code` at `CoroPriority.COMMUNICATION` (60), after the `esp32`
platform has applied user options at priority 1000, so the component's value is the one that lands.

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

Each player row carries a capability int alongside its id and name — bit 1 says the player answers
`media_player.play_media`, bit 2 `media_player.volume_set` — read from Home Assistant's
`supported_features` when the payload is rendered. The routing tree greys out rows without bit 1 and
the ducking tree rows without bit 2, each with a one-line reason, rather than omitting them: a player
silently missing from the list reads as a bug. Selecting a whole area shows a full tick on the area
and on its eligible players, while the greyed rows stay unticked — the call skips them, so showing
them selected would misstate what plays — and the stored selection still says "whole area", which is
what keeps the Route TTS To All Area Players switch in Home Assistant flipping with it. A greyed row
explicitly picked before this existed still shows its tick and can always be *un*ticked — only adding
is blocked. The call-time walks in `tts_routing.yaml` and `area_ducking.yaml` apply the same two bits
when expanding the selection, which matters more than the cosmetics: Home Assistant rejects an
explicit entity list wholesale if one listed entity lacks the service's required feature, so one
incompatible player in a selected area used to be able to silence an announcement for every speaker
in the call.

The consequence worth knowing is that when Home Assistant is unreachable, the app keeps working for
everything local — sensors, LEDs, the radar, the log, maintenance — and greys out only what genuinely
depends on Home Assistant, with a one-line explanation of the problem and the fix.

## The media footer and its three tiers

The media surface is a floating bar under every route, styled after Music Assistant's own mobile
player (September 2026; it replaced the Media card on the home page). The bar wears the album's
colour - the artwork averaged through a tiny canvas, which works because MA's `/imageproxy` sends
CORS headers; a host that refuses pixel reads leaves a theme-coloured bar via CSS `var()` fallbacks.
Artwork and title open the full-screen view, the speaker button (with the group's size as a badge)
slides up a players panel where members are added, removed and mixed, and the bar carries its own
volume row. What the surfaces can do is layered by what can answer, each tier only ever adding to
the one below, and each falling back to it without a seam:

**Tier 0 — the device alone.** The Sendspin hub already receives track metadata and controller state
for the group stream it plays, so `satellite1_web_ui` subscribes (`sendspin_hub_id` in
`common/sendspin.yaml`) and `GET /api/sat1/media` carries title, artist, album, artwork URL,
position, duration, shuffle, repeat and the server's supported-command bitmask alongside the
transport state it always had. Artwork is a URL into Music Assistant's unauthenticated `/imageproxy`,
fetched by the browser — no image bytes ever touch the device. The metadata fragment is pre-escaped
JSON in a mutex-guarded PSRAM string, rebuilt only when the hub reports a change; the numeric state
rides atomics, so the httpd task never blocks the audio path. Shuffle and repeat are commands on
`POST /api/sat1/media` dispatched to the Sendspin player. Seek is the one thing this tier cannot do:
the hub exposes no seek passthrough, so the scrubber is read-only until a higher tier answers.

**Tier 1 — relayed through Home Assistant.** What must round-trip (the favorite button, group
membership, another member's volume, seek) goes through the same machinery as the big Home Assistant
payload: `homeassistant.action` with `capture_response`, riding whichever rung the ladder proved out.
Discovery — which Music Assistant entity is this device, its favorite button, the join candidates —
rides the big payload's `ma` block in `common/web_ui_ha.yaml`, matched by MAC where MA's player id
still embeds one and by name where it does not (MA's universal-player layer mints random ids, so the
name both sides carry is the only surviving key; a device renamed differently on the two sides
discovers nothing and says so). The fast-moving view — group members with volumes — is a separate
small template in `common/web_ui_media.yaml`, synced on demand at `GET /api/sat1/ma` while the
expanded view or the players panel is open (plus one unsynced read at page load, for the bar's
badge), floored at one action call per two seconds however many tabs ask. Commands queue at
`POST /api/sat1/ma/<cmd>` and leave from the main loop, one per iteration; the entity named must
appear in a payload the device itself rendered, which keeps a bug in the app from aiming action
calls at arbitrary entities.

**Tier 2 — the browser talking to Music Assistant directly.** A hand-rolled WebSocket client
(`frontend/src/lib/ma.js`, no npm package) authenticates with a long-lived token against MA's API
and gets real-time player and queue events, true per-member volumes, instant grouping and seek. The
server address and token live in `localStorage` and nowhere else — this tier costs the firmware zero
bytes, and while the socket is up the tier-1 polling stops entirely. The connection panel is folded
shut at the bottom of the sheet; the footer is complete without it.

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
