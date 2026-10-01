/**
 * Local preview for the v2 UI: rebuilds dist-v2/index.html whenever a source file changes, serves it,
 * and forwards everything else to a real device.
 *
 *   DEVICE=http://satellite1-a4c2f8.local npm run dev     # then open http://localhost:5173/
 *
 * Without DEVICE the page still loads, which is enough for layout work against mock data: the static
 * images the firmware serves from flash come from ../assets/ instead, and every API call answers 502.
 *
 * The forwarded Host header is the device's own, so the session gate's DNS-rebinding checks see the
 * name they expect. The setup wizard's writes still refuse to run through here - they also require
 * the device to be un-onboarded and reached over its own access point - so the wizard is tested on
 * the device itself.
 */
import { spawn } from "node:child_process";
import { createServer, request as httpRequest } from "node:http";
import { request as httpsRequest } from "node:https";
import { readFileSync, watch } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const page = join(here, "..", "dist-v2", "index.html");
const port = Number(process.env.PORT || 5173);
const device = process.env.DEVICE ? new URL(process.env.DEVICE) : null;

// The same files web_ui_handler.cpp serves at these paths, embedded by __init__.py.
const assets = join(here, "..", "assets");
const STATIC = {
  "/ui/no-sensor.webp": ["no_sensor.webp", "image/webp"],
  "/ui/icon-192.png": ["icon_192.png", "image/png"],
  "/ui/icon-512.png": ["icon_512.png", "image/png"],
  "/apple-touch-icon.png": ["icon_180.png", "image/png"],
};

let building = null;
let again = false;
function build() {
  if (building) {
    again = true;
    return building;
  }
  building = new Promise((resolve) => {
    const child = spawn(process.execPath, ["build.mjs", "v2"], { cwd: here, stdio: "inherit" });
    child.on("exit", () => {
      building = null;
      if (again) {
        again = false;
        build();
      }
      resolve();
    });
  });
  return building;
}

let debounce;
for (const dir of ["v2", "src", "assets"]) {
  watch(join(here, dir), { recursive: true }, () => {
    clearTimeout(debounce);
    debounce = setTimeout(build, 120);
  });
}

function forward(req, res) {
  if (!device) {
    res.writeHead(502, { "Content-Type": "text/plain" });
    res.end("No DEVICE set - start with DEVICE=http://<device> npm run dev");
    return;
  }
  const send = device.protocol === "https:" ? httpsRequest : httpRequest;
  const upstream = send(
    {
      protocol: device.protocol,
      hostname: device.hostname,
      port: device.port || undefined,
      method: req.method,
      path: req.url,
      headers: { ...req.headers, host: device.host },
    },
    (up) => {
      res.writeHead(up.statusCode ?? 502, up.headers);
      up.pipe(res);
    },
  );
  upstream.on("error", (err) => {
    if (!res.headersSent) res.writeHead(502, { "Content-Type": "text/plain" });
    res.end(String(err));
  });
  req.pipe(upstream);
}

await build();
createServer(async (req, res) => {
  const path = req.url.split("?")[0];
  if (req.method === "GET" && (path === "/" || path === "/ui" || path === "/ui/")) {
    if (building) await building;
    res.writeHead(200, { "Content-Type": "text/html; charset=utf-8", "Cache-Control": "no-store" });
    res.end(readFileSync(page));
    return;
  }
  if (!device && req.method === "GET" && STATIC[path]) {
    const [file, type] = STATIC[path];
    res.writeHead(200, { "Content-Type": type });
    res.end(readFileSync(join(assets, file)));
    return;
  }
  forward(req, res);
}).listen(port, () => {
  console.log(`v2 preview on http://localhost:${port}/` + (device ? ` -> ${device.origin}` : " (no DEVICE: API calls answer 502)"));
});
