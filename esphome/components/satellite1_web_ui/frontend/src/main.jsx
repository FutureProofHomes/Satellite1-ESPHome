/**
 * Entry point.
 *
 * app.css is not imported here. It is a separate esbuild entry point that build.mjs inlines into a
 * <style> block, so importing it would pull it through the JS bundle as well.
 */
import { render } from "preact";

import { App } from "./shell.jsx";

render(<App />, document.getElementById("app"));
