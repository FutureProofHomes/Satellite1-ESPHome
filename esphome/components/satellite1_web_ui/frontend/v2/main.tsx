/**
 * Entry point for the v2 UI.
 *
 * app.css is not imported here: build.mjs builds it as its own entry point and inlines it into a
 * <style> block, so importing it would pull it through the JS bundle as well. The mount point is
 * #root, not v1's #app, because app.css sizes and clips it under that name.
 */
import { render } from 'preact';

import { Satellite1Now } from './components/Satellite1Now';

render(<Satellite1Now />, document.getElementById('root')!);
