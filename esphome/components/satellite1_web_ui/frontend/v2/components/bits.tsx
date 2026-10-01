import { useState } from 'react';

/**
 * Light or dark, remembered per browser under the key v1 used. index.html applies it before the
 * first paint, so this reads the attribute rather than storage - the two cannot disagree if the
 * write ever fails (localStorage throws with site data blocked).
 */
export function useTheme(): ['dark' | 'light', () => void] {
  const [theme, setTheme] = useState<'dark' | 'light'>(() => document.documentElement.dataset.theme === 'light' ? 'light' : 'dark');
  const toggle = () => {
    const next = theme === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    setTheme(next);
    try {
      localStorage.setItem('sat1.theme', next);
    } catch {
      /* The page is already in the right theme; not remembering it is survivable. */
    }
  };
  return [theme, toggle];
}
export function Icon({
  name,
  size = 18
}: {
  name: string;
  size?: number;
}) {
  const paths: Record<string, string> = {
    bell: 'M5 8a4 4 0 0 1 8 0c0 4 2 4 2 5H3c0-1 2-1 2-5Zm3 8h2',
    sun: 'M8 1v2m0 10v2M1 8h2m10 0h2M3 3l1.5 1.5m7 7L13 13M13 3l-1.5 1.5m-7 7L3 13M11 8a3 3 0 1 1-6 0 3 3 0 0 1 6 0Z',
    moon: 'M13.5 9.6A5.8 5.8 0 0 1 6.4 2.5a5.8 5.8 0 1 0 7.1 7.1Z',
    play: 'm6 4 8 4-8 4V4Z',
    pause: 'M6 4v8m4-8v8',
    chevron: 'm5 7 3 3 3-3',
    x: 'm4 4 8 8m0-8-8 8',
    plus: 'M8 3v10M3 8h10',
    search: 'm11 11 3 3M6.8 11a4.2 4.2 0 1 1 0-8.4 4.2 4.2 0 0 1 0 8.4Z',
    mic: 'M8 2a2 2 0 0 1 2 2v4a2 2 0 0 1-4 0V4a2 2 0 0 1 2-2Zm-4 6a4 4 0 0 0 8 0m-4 4v3m-2 0h4'
  };
  return <svg width={size} height={size} viewBox="0 0 16 16" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d={paths[name] || paths.plus} /></svg>;
}
/** Home Assistant's cog, drawn inline in the walk-through's step 2 where the person has to find it. */
const Cog = () => <svg className="fix-cog" viewBox="0 0 24 24" aria-hidden="true"><path d="M12,15.5A3.5,3.5 0 0,1 8.5,12A3.5,3.5 0 0,1 12,8.5A3.5,3.5 0 0,1 15.5,12A3.5,3.5 0 0,1 12,15.5M19.43,12.97C19.47,12.65 19.5,12.33 19.5,12C19.5,11.67 19.47,11.34 19.43,11L21.54,9.37C21.73,9.22 21.78,8.95 21.66,8.73L19.66,5.27C19.54,5.05 19.27,4.96 19.05,5.05L16.56,6.05C16.04,5.66 15.5,5.32 14.87,5.07L14.5,2.42C14.46,2.18 14.25,2 14,2H10C9.75,2 9.54,2.18 9.5,2.42L9.13,5.07C8.5,5.32 7.96,5.66 7.44,6.05L4.95,5.05C4.73,4.96 4.46,5.05 4.34,5.27L2.34,8.73C2.21,8.95 2.27,9.22 2.46,9.37L4.57,11C4.53,11.34 4.5,11.67 4.5,12C4.5,12.33 4.53,12.65 4.57,12.97L2.46,14.63C2.27,14.78 2.21,15.05 2.34,15.27L4.34,18.73C4.46,18.95 4.73,19.03 4.95,18.95L7.44,17.94C7.96,18.34 8.5,18.68 9.13,18.93L9.5,21.58C9.54,21.82 9.75,22 10,22H14C14.25,22 14.46,21.82 14.5,21.58L14.87,18.93C15.5,18.67 16.04,18.34 16.56,17.94L19.05,18.95C19.27,19.03 19.54,18.95 19.66,18.73L21.66,15.27C21.78,15.05 21.73,14.78 21.54,14.63L19.43,12.97Z" /></svg>;

/**
 * Step 2 of the actions walk-through: the copy.js template's %c becomes the cog and %s the device's
 * name. The gate and the setup wizard both render it, so the two always speak the same steps.
 */
export const cogStep = (tpl: string, name: string) => {
  const [before, after] = tpl.split('%c');
  return <>{before}<Cog />{after.replace('%s', name)}</>;
};

export const Logo = ({
  cls = 'login-logo'
}: {
  cls?: string;
}) => <svg className={cls} viewBox="0 0 79.375 79.375" fill="none" stroke="currentColor" strokeLinecap="square" aria-hidden="true">
    <g transform="matrix(1.6754,0,0,1.6754,84.9754,-16.4554)" strokeWidth="1.31">
      <path d="m -45.44,31.52 11,-10.96 11,10.96 v 16.25 l -5.82,.01" />
      <path d="m -27.49,20.19 10.94,9.29 v 18.3 l 8.35,.03 V 29.29 l -10.94,-9.66 -1.92,1.73" />
      <path strokeWidth="1.36" d="m -32.25,47.75 c 0,-7.27 -5.89,-13.42 -13.16,-13.42 h 0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -36.31,47.73 c .01,-.12 .01,-.12 .01,-.24 0,-5.03 -4.08,-9.11 -9.11,-9.11 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -40.36,47.8 c .01,-.12 .01,-.18 .01,-.31 0,-2.8 -2.27,-5.06 -5.06,-5.06 l 0,0 c -.05,0 -.1,0 -.15,.01" />
      <path strokeWidth="1.36" d="m -45.41,46.48 a 1.01,1.01 0 0 0 -.15,.01 v 1.38 h 1.09 a 1.01,1.01 0 0 0 .07,-.37 1.01,1.01 0 0 0 -1.01,-1.01 z" />
    </g>
  </svg>;
