/**
 * The lucide icons the design uses, inlined so the bundle carries their paths and nothing else.
 * Path data is lucide-static 1.49.0 (ISC). Props follow lucide-react's: size, strokeWidth, color, and
 * anything else passed straight to the <svg>.
 */
import type { ComponentChildren, JSX } from 'preact';

type IconProps = Omit<JSX.SVGAttributes<SVGSVGElement>, 'size'> & {
  size?: number | string;
  strokeWidth?: number | string;
  color?: string;
};

function icon(name: string, paths: ComponentChildren) {
  return function Icon({ size = 24, strokeWidth = 2, color = 'currentColor', className, ...rest }: IconProps) {
    return (
      <svg
        xmlns="http://www.w3.org/2000/svg"
        width={size}
        height={size}
        viewBox="0 0 24 24"
        fill="none"
        stroke={color}
        stroke-width={strokeWidth}
        stroke-linecap="round"
        stroke-linejoin="round"
        class={className ? `lucide lucide-${name} ${className}` : `lucide lucide-${name}`}
        {...rest}
      >
        {paths}
      </svg>
    );
  };
}

export const AlertTriangle = icon('triangle-alert', <>
  <path d="m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3" />
  <path d="M12 9v4" />
  <path d="M12 17h.01" />
</>);

export const Info = icon('info', <>
  <circle cx="12" cy="12" r="10" />
  <path d="M12 16v-4" />
  <path d="M12 8h.01" />
</>);

export const XCircle = icon('circle-x', <>
  <circle cx="12" cy="12" r="10" />
  <path d="m15 9-6 6" />
  <path d="m9 9 6 6" />
</>);

export const Clock = icon('clock', <>
  <circle cx="12" cy="12" r="10" />
  <path d="M12 6v6l4 2" />
</>);

export const X = icon('x', <>
  <path d="M18 6 6 18" />
  <path d="m6 6 12 12" />
</>);

export const LogOut = icon('log-out', <>
  <path d="m16 17 5-5-5-5" />
  <path d="M21 12H9" />
  <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4" />
</>);

export const ChevronDown = icon('chevron-down', <path d="m6 9 6 6 6-6" />);

export const ArrowUp = icon('arrow-up', <>
  <path d="m5 12 7-7 7 7" />
  <path d="M12 19V5" />
</>);

export const RadarIcon = icon('radar', <>
  <path d="M19.07 4.93A10 10 0 0 0 6.99 3.34" />
  <path d="M4 6h.01" />
  <path d="M2.29 9.62A10 10 0 1 0 21.31 8.35" />
  <path d="M16.24 7.76A6 6 0 1 0 8.23 16.67" />
  <path d="M12 18h.01" />
  <path d="M17.99 11.66A6 6 0 0 1 15.77 16.67" />
  <circle cx="12" cy="12" r="2" />
  <path d="m13.41 10.59 5.66-5.66" />
</>);

export const Mic = icon('mic', <>
  <path d="M12 19v3" />
  <path d="M19 10v2a7 7 0 0 1-14 0v-2" />
  <rect x="9" y="2" width="6" height="13" rx="3" />
</>);

export const MicOff = icon('mic-off', <>
  <path d="M12 19v3" />
  <path d="M15 9.34V5a3 3 0 0 0-5.68-1.33" />
  <path d="M16.95 16.95A7 7 0 0 1 5 12v-2" />
  <path d="M18.89 13.23A7 7 0 0 0 19 12v-2" />
  <path d="m2 2 20 20" />
  <path d="M9 9v3a3 3 0 0 0 5.12 2.12" />
</>);

export const ArrowRight = icon('arrow-right', <>
  <path d="M5 12h14" />
  <path d="m12 5 7 7-7 7" />
</>);

export const Check = icon('check', <path d="M20 6 9 17l-5-5" />);

export const Activity = icon('activity', <path d="M22 12h-2.48a2 2 0 0 0-1.93 1.46l-2.35 8.36a.25.25 0 0 1-.48 0L9.24 2.18a.25.25 0 0 0-.48 0l-2.35 8.36A2 2 0 0 1 4.49 12H2" />);

export const Plus = icon('plus', <>
  <path d="M5 12h14" />
  <path d="M12 5v14" />
</>);

export const ChevronRight = icon('chevron-right', <path d="m9 18 6-6-6-6" />);

export const ChevronLeft = icon('chevron-left', <path d="m15 18-6-6 6-6" />);

export const House = icon('house', <>
  <path d="M15 21v-8a1 1 0 0 0-1-1h-4a1 1 0 0 0-1 1v8" />
  <path d="M3 10a2 2 0 0 1 .709-1.528l7-5.999a2 2 0 0 1 2.582 0l7 5.999A2 2 0 0 1 21 10v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z" />
</>);

export const AudioLines = icon('audio-lines', <>
  <path d="M2 10v3" />
  <path d="M6 6v11" />
  <path d="M10 3v18" />
  <path d="M14 8v7" />
  <path d="M18 5v13" />
  <path d="M22 10v3" />
</>);

export const Volume2 = icon('volume-2', <>
  <path d="M11 4.702a.705.705 0 0 0-1.203-.498L6.413 7.587A1.4 1.4 0 0 1 5.416 8H3a1 1 0 0 0-1 1v6a1 1 0 0 0 1 1h2.416a1.4 1.4 0 0 1 .997.413l3.383 3.384A.705.705 0 0 0 11 19.298z" />
  <path d="M16 9a5 5 0 0 1 0 6" />
  <path d="M19.364 18.364a9 9 0 0 0 0-12.728" />
</>);

export const Settings = icon('settings', <>
  <path d="M12.22 2h-.44a2 2 0 0 0-2 2v.18a2 2 0 0 1-1 1.73l-.43.25a2 2 0 0 1-2 0l-.15-.08a2 2 0 0 0-2.73.73l-.22.38a2 2 0 0 0 .73 2.73l.15.1a2 2 0 0 1 1 1.72v.51a2 2 0 0 1-1 1.74l-.15.09a2 2 0 0 0-.73 2.73l.22.38a2 2 0 0 0 2.73.73l.15-.08a2 2 0 0 1 2 0l.43.25a2 2 0 0 1 1 1.73V20a2 2 0 0 0 2 2h.44a2 2 0 0 0 2-2v-.18a2 2 0 0 1 1-1.73l.43-.25a2 2 0 0 1 2 0l.15.08a2 2 0 0 0 2.73-.73l.22-.39a2 2 0 0 0-.73-2.73l-.15-.08a2 2 0 0 1-1-1.74v-.5a2 2 0 0 1 1-1.74l.15-.09a2 2 0 0 0 .73-2.73l-.22-.38a2 2 0 0 0-2.73-.73l-.15.08a2 2 0 0 1-2 0l-.43-.25a2 2 0 0 1-1-1.73V4a2 2 0 0 0-2-2z" />
  <circle cx="12" cy="12" r="3" />
</>);

export const Sun = icon('sun', <>
  <circle cx="12" cy="12" r="4" />
  <path d="M12 2v2" />
  <path d="M12 20v2" />
  <path d="m4.93 4.93 1.41 1.41" />
  <path d="m17.66 17.66 1.41 1.41" />
  <path d="M2 12h2" />
  <path d="M20 12h2" />
  <path d="m6.34 17.66-1.41 1.41" />
  <path d="m19.07 4.93-1.41 1.41" />
</>);

export const Moon = icon('moon', <path d="M12 3a6 6 0 0 0 9 9 9 9 0 1 1-9-9Z" />);

/** Auto: a circle half filled, light and dark sharing it. Not lucide's; drawn to sit beside Sun and Moon. */
export const CircleHalf = icon('circle-half', <>
  <circle cx="12" cy="12" r="9" />
  <path d="M12 3a9 9 0 0 1 0 18z" fill="currentColor" />
</>);
