/**
 * Entry point.
 *
 * app.css is not imported here: build.mjs builds it as its own entry point and inlines it into a
 * <style> block, so importing it would pull it through the JS bundle as well. The mount point is
 * #root because app.css sizes and clips it under that name.
 */
import { render } from 'preact';

import { App } from './App';

render(<App />, document.getElementById('root')!);
