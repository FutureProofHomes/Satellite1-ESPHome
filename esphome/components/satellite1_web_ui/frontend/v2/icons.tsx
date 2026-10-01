/**
 * The twelve lucide icons the design uses, inlined so the bundle carries their paths and nothing else.
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
