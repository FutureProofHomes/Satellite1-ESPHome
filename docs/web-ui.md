# The on-device web app

Satellite1 serves its own configuration app from the device, at `http://<device>/`. It needs no
internet connection, no CDN and no Home Assistant to load — everything it needs is in flash.

### iOS Safari and the "This Connection Is Not Private" interstitial

This one was chased in the wrong direction before it was understood, so the conclusion is worth
stating plainly: **it is not a port problem and not something the firmware can fix.** With iCloud
Private Relay on, iOS Safari shows a full-page "This Connection Is Not Private" interstitial before it
will connect to a cleartext-HTTP destination that is **on a different subnet from the iPhone**.
Private Relay leaves the phone's own local subnet alone and engages for everything else; for
off-subnet HTTP to a private IP it cannot protect, it warns.

It was pinned down on hardware across eight cases. With the Satellite1 and Home Assistant on the same
scheme and port, Home Assistant — on the same subnet as the phone — loaded clean, while the Satellite1
— on a separate IoT VLAN — warned. IP versus `.local` made no difference, and neither did moving the
server's port from 80 to 8080 to 8123 (an earlier theory blamed the port; it was a red herring, the
port and the host had simply moved together in the first tests). Because the interstitial is shown
*before any byte is exchanged*, nothing the server does — headers, HTML, TLS, the login page — can
change it; Home Assistant serves the same cleartext HTTP and only escapes the warning by being local
to the phone. macOS Safari warned on none of the cases; this is iOS-only. So the app is served on port
80, the friendly default, and the interstitial is addressed at the network layer instead:

- **Turn off "Limit IP Address Tracking" for that Wi-Fi network** (Settings → Wi-Fi → the network).
  This is the per-network Private Relay toggle, immediate, and does not disable Private Relay
  elsewhere. The practical answer for a segmented network.
- **Put the phone and the device on the same subnet.** Makes the device local to Private Relay, at
  the cost of the IoT-VLAN separation.
- **Front the device with an HTTPS reverse proxy** (see the reverse-proxy note under Authentication).
  The only way to keep Private Relay on, stay cross-subnet, and load clean, because it is then real
  HTTPS.
- **The installed home-screen PWA is worth testing here.** Private Relay governs Safari browsing only;
  a standalone home-screen web app runs outside it, so the app may load clean cross-subnet once
  installed — even though the one-time "add to home screen" in Safari still hits the interstitial.

The `.local` smart redirect is easy to over-credit in this context: it is JavaScript in the bundle, so
it runs only *after* the page loads, and cannot prevent a pre-page-load interstitial. It exists for
origin stability — a session cookie on the `.local` origin survives DHCP giving the device a new IP —
not as a Private Relay workaround. For browsers that cannot resolve mDNS at all (Android Chrome, the
HA app's webview), a failed probe is remembered per-origin for a day, so the 2.5-second timeout is
paid once rather than on every visit, and a static boot splash covers the wait — the page is never
blank while the probe runs.

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
| `GET /api/sat1/login/nonce`, `POST /api/sat1/login` | Challenge-response sign-in; also accepts a `key` |
| `POST /api/sat1/login/start`, `GET /api/sat1/login/poll` | The device-presence pairing window |
| `POST /api/sat1/logout`, `POST /api/sat1/logout_all` | Expire this cookie; revoke every session |
| `GET /api/sat1/whoami` | The mDNS hostname, the one deliberately public endpoint |
| `/manifest.webmanifest`, `/ui/icon-*.png`, `/apple-touch-icon.png` | The PWA surface |
| `/api/v1/...` | The radar's own API, served by `satellite1_radar` |

The login and PWA endpoints, the SPA itself and `/ui/no-sensor.webp` are the only unauthenticated
paths; `whoami` returns nothing but the hostname, which the pre-login smart redirect needs.
Everything with device state in it — `GET /api/sat1/state` included — stays behind the gate, because
its readings and voice transcripts are exactly what an unauthenticated surface would hand a
DNS-rebinding page (a rebound hostname never carries the host-bound cookie). The pairing-window
endpoints (`login/start`, `login/poll`, `login/cancel`) additionally require a `Host` header the
device answers to — any IP literal, or the device's own mDNS name — which closes the rebinding play
against the flow itself: a rebound page necessarily carries the attacker's registered hostname, and
neither a dotted-quad nor a `.local` name can be registered in public DNS. Password and `?key=`
logins are exempt from the Host check (they already demand a secret a rebound page does not have),
so signing in through a reverse proxy that forwards its own `Host` still works — by password.

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

Every request — the app, the entity API, the radar API, `/events` — passes through a session gate
(`session_gate.cpp`) that `satellite1_web_ui` registers ahead of every other handler on the shared
server. `web_server` no longer carries an `auth:` block, and must not: it would wrap every later
handler in ESPHome's digest middleware *behind* the gate's own cookie check and lock out every
signed-in browser. The component's config validation refuses that combination with an error that says
so.

A browser signs in once and holds a 90-day `HttpOnly` cookie, which is what ends the iOS misery of a
credential prompt on every visit (iOS never saves HTTP-auth logins). The session token is
`HMAC-SHA256(nvs_salt, password || generation)`: a per-device random salt in NVS, keyed by the
password and a generation counter. Changing the password invalidates every session; so does bumping
the generation — the "Sign out everywhere" action on Diagnostics — which is the only way to revoke a
leaked sign-in link or a stolen cookie without also changing the password. The cookie's `Max-Age`
only expires the browser's own copy; the token value stays valid until the password or generation
moves, which is why that action exists.

The username and password are still set the same way — the username fixed in YAML, the password
defaulting to a value derived from the device MAC, overridable with a `web_ui_password` substitution,
and published as the **Web UI Password** diagnostic sensor because that is the only way a customer
discovers the generated value. They now go to `satellite1_web_ui` at boot rather than to
`web_server`, and they double as HTTP **digest** credentials: the gate runs the same digest check
ESPHome used to, so `curl --digest -u satellite1:...` and the radar tuner scripts keep working
unchanged. The digest challenge (`WWW-Authenticate`) is sent only to clients that look like scripts
— requests whose `User-Agent` does not begin with `Mozilla/`, which every browser's does and no CLI
tool's does — because digest cannot be spoken without the challenge, but handing it to a browser
would pop the native credentials dialog over the app's own `fetch()` calls, the exact prompt the
cookie session removes. (`Sec-Fetch-Mode` would be the principled discriminator, but browsers only
send Fetch Metadata headers to HTTPS origins, and this device is plain HTTP; a script that fakes a
browser UA simply forgoes digest and uses the cookie or `?key=` flow like a browser would.)

### Ways to sign in

- **Password.** Typed into the login page. It is challenge-response — the device issues a single-use
  nonce and the browser answers `HMAC-SHA256(SHA-256(password), nonce)` — so the password itself
  never crosses the wire, the one property Basic auth lacked. (`crypto.subtle` is unavailable on
  insecure origins, so the app carries its own small SHA-256; see `frontend/src/lib/auth.js`.)
- **On the device.** "Sign in on this device" opens a 60-second pairing window and the device
  approves it by physical presence, so nothing is typed. When Home Assistant is connected the device
  speaks a random four-digit code and opens its mic — say the code back. When it is not, the device
  speaks a challenge built from its own wake words ("Hey Jarvis, Hey Jarvis, Stop") and you repeat
  them in order; this path is fully offline. Either way a press of the **action button** approves the
  window instead. Muted microphones fall back to the button alone, and the login page says which
  applies. This is the flow for a ceiling-mounted device nobody can reach to type on.
- **Sign-in link and QR.** Diagnostics → Launch shows a tokenized URL and a QR carrying the same
  key. Scan the QR with a phone and it lands signed in with no typing; paste the link into a Home
  Assistant dashboard button and it becomes a true launch button. The two are built on different
  origins, each matched to its lifetime: the QR uses the device's current IP, because it is scanned
  live off the screen (the address is fresh by construction) and an IP works on every phone where a
  `.local` QR is a dead end for mDNS-less ones — phones that *can* resolve mDNS still end up on
  `.local`, since the smart redirect carries `?key=` along. The copyable link uses the permanent
  `.local` name, which survives DHCP churn — the right form for anything long-lived. If a dashboard
  button must use an IP (say, for a browser that cannot resolve `.local`), give the device a DHCP
  reservation first, or the button dies with the device's next lease. The link is a bearer
  credential — anyone who has it can sign in — so treat it like the password, and use "Sign out
  everywhere" to revoke it. If you paste it into a dashboard, the key then lives in that dashboard's
  configuration, readable by anyone who can edit dashboards.

Home-screen note for iOS: an app added to the home screen has its own cookie store, separate from
Safari's, so its first launch shows the login page once even if Safari was already signed in — one
approval and the installed app holds its own 90-day cookie. Android home-screen shortcuts open in
Chrome proper and share its cookies, so they are signed in immediately.

### The wire, honestly

Two channels, and they are not the same. The native API to Home Assistant is Noise-encrypted
(`api: encryption:` in `common/home_assistant.yaml`), so everything the device exchanges with Home
Assistant is protected. The web server is plain HTTP — ESPHome offers no TLS there — and this design
works within that: the password never crosses the wire, and the presence sign-ins never transmit a
secret at all, but the session cookie and the sign-in key are bearer tokens a LAN sniffer could
replay. That is unfixable without TLS. On-device HTTPS is deliberately not attempted: no public CA
issues certificates for private IPs or `.local` names, so it would be self-signed — a permanent,
scarier browser warning than the cross-subnet Private Relay interstitial (see the interstitial note
near the top) — and it would cost TLS handshake memory on a chip already running audio pipelines.

Anyone who wants encrypted browser access puts a reverse proxy with a real certificate in front of
the device — the Nginx Proxy Manager or Caddy add-on in Home Assistant with an internal domain and a
Let's Encrypt DNS-01 wildcard, or a Tailscale node, which issues real HTTPS certificates for tailnet
hosts. No firmware change is needed, and it composes with everything here: the session cookie simply
binds to the proxy's hostname.

## The bundle

Measured, from the build that produced this note (September 2026, after the full visual-polish
pass: motion language, the redesigned nav drawer, the radar hero treatment, media finishing, the
type ramp, empty states, the brand gradient (drawer indicator and slider fills only - never on
buttons, by the owner's call), header blur, tree chips, first-load shimmers and the
desktop two-column layout - all eleven rounds plus the request-queue deadline, plus the unified
drawer pattern that followed: swipe-to-dismiss, one-drawer-at-a-time, and the switcher and
expanded media view dressed as drawers):

| | Bytes |
|---|---|
| Raw HTML document | 148,835 |
| Gzipped, as embedded | **49,367** |
| Budget | 51,200 |

That is 96% of the ceiling. The budget has been raised twice: from 40,960 B when the media footer
landed at 99.8% of it, to 48 KB for the visual-polish pass, and to 50 KB (with the owner's approval)
when the mobile sign-in work landed — the login screen, the challenge-response SHA-256/HMAC and the
QR encoder cost about 7 KB together, and the parts that could never run were trimmed first (QR
versions past 6, which nothing this encodes can reach, and the Inkscape-precision decimals in the
inlined logo). The number is a self-imposed discipline rather than a hardware limit — flash sits
under 46% used either way. The gzip figure is the one that matters, since that is what occupies flash
and what crosses the network. It is produced by `gzip.compress(html, compresslevel=9)` in the
component's codegen, so the number reported at compile time is the number that ships.

`dist/index.html` is committed, and CI rebuilds it and fails on `git diff --exit-code` against `dist`,
so the committed bundle cannot drift from the source it was built from. To rebuild locally (`npm ci`,
not `npm install`, to match the pinned toolchain the drift check builds against):

```
cd esphome/components/satellite1_web_ui/frontend
npm ci
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
what keeps the Route TTS To All Area Players switch in Home Assistant flipping with it. Permanently
ineligible rows — an incompatible player or this device's own media player — are always shown
unticked and disabled, even if an old explicit selection still names one. The call-time walks in
`tts_routing.yaml` and `area_ducking.yaml` apply the same capability tests and reject this device
itself when expanding the selection, which matters more than the cosmetics: Home Assistant rejects
an explicit entity list wholesale if one listed entity lacks the service's required feature, so one
incompatible player in a selected area used to be able to silence an announcement for every speaker
in the call.

Each row also carries an availability flag: 0 when Home Assistant reported the player `unavailable`
or `unknown` at render time. An offline row greys out and gains an "Offline" marker in both trees,
but — unlike a missing capability — its checkbox keeps working and a prior tick stays ticked, because
being offline is transient and the selection is a setting its owner still wants when the speaker
returns. The call-time walk in `tts_routing.yaml` applies the same state test, so no routing, chime,
stop or volume call targets an offline player; ducking's walk already rejected unavailable states.
Availability is as fresh as the cached payload — synced on connect and once per page load — so a
player that drops while the page is open shows stale until reload, like every other fact in it.

The consequence worth knowing is that when Home Assistant is unreachable, the app keeps working for
everything local — sensors, LEDs, the radar, the log, maintenance — and greys out only what genuinely
depends on Home Assistant, with a one-line explanation of the problem and the fix.

## The actions verdict and the onboarding splash

The single most common broken state is none of the above: Home Assistant connected and healthy, but
the "Allow the device to perform Home Assistant actions" checkbox unticked — which it is for every
newly added ESPHome device. Every payload above rides an action call, so a blocked device serves an
empty `GET /api/sat1/ha` forever, and until September 2026 the app could only hedge ("too old, or
not allowed") because a refused call and an unsupported one look identical from the response.

The hedge is gone because the device already knows the difference: `tts_routing.yaml`'s probe
concludes `ha_actions_allowed` (0 unknown, 1 allowed, 2 blocked, 3 unverifiable — a pre-2025.12 Home
Assistant answers no action call, so silence must not accuse the checkbox). The probe runs on every
Home Assistant connect — it was gated on an active routing selection until a live blocked test
device (September 18 2026) showed the splash "asking" forever because nothing had ever concluded the
verdict. The same test exposed that the sync ladder in `web_ui_ha.yaml` could not see a refusal
either: a blocked checkbox makes Home Assistant drop the call *silently*, so the ladder's `on_error`
never fires — it now carries a 10s silence watchdog that records `rung: -1` when no rung answers.
The verdict is pushed into `satellite1_web_ui` from `tts_routing_status_publish` — the one script
every transition runs through — and rides `GET /api/sat1/ha` as `actions`, next to the `rung` it
disambiguates.

What the app builds on it (`frontend/src/splash.jsx`): a verdict overlay at the first authenticated
moment — mounted with the app, right after login hands over or immediately for a returning cookie —
that holds the reveal while the boot calls land. Happy path: a sub-second fade. Blocked: the
checkbox walk-through, quoted verbatim, with a My Home Assistant deep link to the ESPHome
integration page and a 3s watch on the device's cache — ticking the box makes Home Assistant reload
the config entry, which re-syncs the payload, so the screen melts into the app on its own. Not
connected, too old, slow and device-error each get their own card, and every card carries "Continue
without Home Assistant" into the degraded app, because the device's own controls owe nothing to any
of them. The same walk-through reappears as a bottom drawer (the app's shared drawer pattern),
opened by a one-time toast nudge after Continue and by "Show fix" links beside every blocked
caption, so the fix stays one tap away from wherever the absence is felt.

The same pass retired the amber banners app-wide (owner decision): the stream-lost banner became the
one sticky toast on the shared toast surface at the bottom edge (transients — a failed write, the
blocked nudge — stack above it), the selection-write banner folded into the ordinary write-failed
toast, and the Audio cards' data-layer states became quiet dim captions. `.banner` survives only in
its neutral form, on Diagnostics.

## The media footer and its three tiers

The media surface is a floating bar under every route, styled after Music Assistant's own mobile
player (September 2026; it replaced the Media card on the home page). The bar wears the album's
colour - the artwork averaged through a tiny canvas, which works because MA's `/imageproxy` sends
CORS headers; a host that refuses pixel reads leaves a theme-coloured bar via CSS `var()` fallbacks.
Artwork and title open the expanded view, the speaker button (with the group's size as a badge)
slides up a players panel where members are added, removed and mixed, and the bar carries its own
volume row. Both surfaces - and the device switcher in the header - are drawers cut from the same
pattern (September 2026): rounded shoulders, a grab handle, a scrim with the page showing through,
a swipe on the handle or label row that follows the finger and dismisses past a threshold
(`useSheetDrag` in `ui.jsx`), and an app-wide one-drawer-at-a-time rule - opening any drawer,
the nav pane included, closes whichever other one is standing (`useDrawer`, same file). The
handle's visible pill is 42x5 but its hit area is 148x32 (sticky and full-width at the switcher's
bottom edge), and while any drawer stands the page behind is frozen (`html.held`) - both fixes for
iOS Safari swipes that missed the old 5px handle and scrolled the document instead. What the surfaces can do is layered by what can answer, each tier only ever adding to
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

## Add to home screen

The app ships a web manifest, three PNG icons (192, 512 and a 180px apple-touch-icon) and the iOS
standalone meta tags, all embedded in flash and served from PROGMEM like the no-sensor photo, so they
cost no heap. Added to a phone's home screen, the app opens full-screen without Safari's chrome, and
its cookie keeps it signed in. Because a standalone home-screen app runs outside Safari's browsing
context, it should also escape the iCloud Private Relay interstitial that a cross-subnet device draws
in Safari itself (see the interstitial note near the top) — worth confirming on a segmented network,
since the one-time "add to home screen" still happens in Safari and hits it once.

On iOS this is a real standalone web app with its own cookie store, so its first launch asks for one
sign-in even if Safari was already signed in (see the home-screen note under Authentication). On
Android over plain HTTP the browser makes a home-screen *shortcut* rather than an installed PWA —
`display: standalone` needs a secure context — which carries the same icon and name and shares
Chrome's cookies, so it launches straight in. The icons are drawn from the FutureProofHomes mark on
the app's dark background, so the home-screen icon, the splash and the app agree.

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
