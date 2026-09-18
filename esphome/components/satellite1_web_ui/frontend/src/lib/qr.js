/**
 * A QR encoder just big enough for the sign-in link, and nothing else.
 *
 * Byte mode, error-correction level L, versions 1-6, and a fixed mask. Version 6's 134 characters
 * is not a guess at enough: the link is 88 fixed characters plus the hostname, and ESPHome caps
 * hostnames at 31, so the worst case is 119. Fixing the mask instead of scoring all eight is the
 * one liberty taken with the spec: mask choice is an encoder-side optimization for marginal scan
 * conditions, any mask is legal, and every decoder reads them all - it buys back the largest
 * single chunk of code in a full encoder. Verified round-trip against an independent decoder
 * (jsQR) across versions before shipping.
 *
 * Renders to an SVG path string, because the login and Diagnostics surfaces are SVG already and a
 * canvas would be the only one in the app.
 */

/* GF(256) with the QR polynomial 0x11D, tables built once at load. */
const EXP = new Uint8Array(512);
const LOG = new Uint8Array(256);
for (let i = 0, x = 1; i < 255; i++) {
  EXP[i] = x;
  LOG[x] = i;
  x <<= 1;
  if (x & 0x100) x ^= 0x11d;
}
for (let i = 255; i < 512; i++) EXP[i] = EXP[i - 255];
const gmul = (a, b) => (a && b ? EXP[LOG[a] + LOG[b]] : 0);

/** Reed-Solomon remainder of `data` against the generator polynomial of degree `ec`. */
function rsRemainder(data, ec) {
  // Generator = product of (x - 2^i) for i in [0, ec).
  let gen = [1];
  for (let i = 0; i < ec; i++) {
    const next = new Array(gen.length + 1).fill(0);
    for (let j = 0; j < gen.length; j++) {
      next[j] ^= gmul(gen[j], EXP[i]);
      next[j + 1] ^= gen[j];
    }
    gen = next;
  }
  gen.reverse(); // Highest degree first, matching the long division below.

  const rem = new Uint8Array(ec);
  for (const byte of data) {
    const factor = byte ^ rem[0];
    rem.copyWithin(0, 1);
    rem[ec - 1] = 0;
    if (factor) {
      for (let i = 0; i < ec; i++) rem[i] ^= gmul(gen[i + 1], factor);
    }
  }
  return rem;
}

/* Level L geometry for versions 1-6: total data codewords, EC codewords per block, block count,
 * and the alignment-pattern centers. Single blocks through v5, two equal blocks for v6. Stops at
 * v6 because nothing this encodes can outgrow it - see the header - and v7 is where version-info
 * blocks and triple alignment rows begin, a chunk of encoder nothing would ever run. */
const VERSIONS = [
  { data: 19, ec: 7, blocks: 1, align: [] },
  { data: 34, ec: 10, blocks: 1, align: [6, 18] },
  { data: 55, ec: 15, blocks: 1, align: [6, 22] },
  { data: 80, ec: 20, blocks: 1, align: [6, 26] },
  { data: 108, ec: 26, blocks: 1, align: [6, 30] },
  { data: 136, ec: 18, blocks: 2, align: [6, 34] },
];

/** 15-bit format info for EC level L and `mask`: BCH(15,5) with the spec's fixed XOR mask. */
function formatBits(mask) {
  const data = (0b01 << 3) | mask; // L = 01
  let bch = data << 10;
  for (let i = 14; i >= 10; i--) {
    if (bch & (1 << i)) bch ^= 0x537 << (i - 10);
  }
  return ((data << 10) | bch) ^ 0x5412;
}

const MASK = 0; // (row + col) % 2 === 0

/**
 * Encodes `text` and returns { size, path }: the module count per side and an SVG path drawing
 * every dark module as a unit square, for a viewBox of `0 0 size size`.
 */
export function qrSvgPath(text) {
  const bytes = new TextEncoder().encode(text);

  // Smallest version that fits: header is 4 bits mode + 8 bits count (v1-9), so capacity in
  // characters is total data codewords minus 2.
  const vi = VERSIONS.findIndex((v) => bytes.length <= v.data - 2);
  if (vi < 0) return null;
  const spec = VERSIONS[vi];
  const version = vi + 1;
  const size = 17 + version * 4;

  /* --- Bit stream: mode, count, data, terminator, padding --- */
  const stream = new Uint8Array(spec.data);
  let bitAt = 0;
  const push = (value, bits) => {
    for (let i = bits - 1; i >= 0; i--, bitAt++) {
      if ((value >> i) & 1) stream[bitAt >> 3] |= 0x80 >> (bitAt & 7);
    }
  };
  push(0b0100, 4);
  push(bytes.length, 8);
  for (const b of bytes) push(b, 8);
  // Terminator and byte alignment are zeros the array already holds; fill the rest with the
  // spec's alternating pad codewords.
  let padAt = Math.ceil((bitAt + 4) / 8);
  for (let flip = 0; padAt < spec.data; padAt++, flip ^= 1) stream[padAt] = flip ? 0x11 : 0xec;

  /* --- Split into blocks, append RS codewords, interleave --- */
  const per = spec.data / spec.blocks;
  const dataBlocks = [];
  const ecBlocks = [];
  for (let b = 0; b < spec.blocks; b++) {
    const slice = stream.slice(b * per, (b + 1) * per);
    dataBlocks.push(slice);
    ecBlocks.push(rsRemainder(slice, spec.ec));
  }
  const codewords = [];
  for (let i = 0; i < per; i++) for (const blk of dataBlocks) codewords.push(blk[i]);
  for (let i = 0; i < spec.ec; i++) for (const blk of ecBlocks) codewords.push(blk[i]);

  /* --- The matrix. `fixed` marks function modules the data walk must step around. --- */
  const dark = new Uint8Array(size * size);
  const fixed = new Uint8Array(size * size);
  const set = (r, c, on) => {
    dark[r * size + c] = on ? 1 : 0;
    fixed[r * size + c] = 1;
  };

  // A finder with its separator ring: dark center 3x3 and dark ring at distance 3, light between
  // and around, drawn one module past the 7x7 so the separator comes for free.
  const finder = (r0, c0) => {
    for (let r = -1; r <= 7; r++) {
      for (let c = -1; c <= 7; c++) {
        const rr = r0 + r;
        const cc = c0 + c;
        if (rr < 0 || cc < 0 || rr >= size || cc >= size) continue;
        const ring = Math.max(Math.abs(r - 3), Math.abs(c - 3));
        set(rr, cc, ring <= 1 || ring === 3);
      }
    }
  };
  finder(0, 0);
  finder(0, size - 7);
  finder(size - 7, 0);

  // Timing patterns.
  for (let i = 8; i < size - 8; i++) {
    set(6, i, i % 2 === 0);
    set(i, 6, i % 2 === 0);
  }

  // Alignment patterns: every pair of centers except the three that would overlap a finder.
  for (const r of spec.align) {
    for (const c of spec.align) {
      const nearFinder = (r < 8 && c < 8) || (r < 8 && c > size - 9) || (r > size - 9 && c < 8);
      if (nearFinder) continue;
      for (let dr = -2; dr <= 2; dr++) {
        for (let dc = -2; dc <= 2; dc++) {
          set(r + dr, c + dc, Math.max(Math.abs(dr), Math.abs(dc)) !== 1);
        }
      }
    }
  }

  // The dark module, and the format info in both homes. The two copies each straddle regions -
  // the vertical run splits between the top-left and bottom-left finders, the horizontal one
  // between top-left and top-right - and the split points are the spec's, bit 0 the LSB.
  set(size - 8, 8, 1);
  const fmt = formatBits(MASK);
  for (let i = 0; i < 15; i++) {
    const bit = (fmt >> i) & 1;
    // Vertical copy, column 8: rows 0-5, 7, 8, then the bottom seven.
    if (i < 6) set(i, 8, bit);
    else if (i === 6) set(7, 8, bit);
    else if (i === 7) set(8, 8, bit);
    else set(size - 15 + i, 8, bit);
    // Horizontal copy, row 8: the right eight, then column 7, then columns 5-0.
    if (i < 8) set(8, size - 1 - i, bit);
    else if (i === 8) set(8, 7, bit);
    else set(8, 14 - i, bit);
  }

  /* --- Data placement: right-to-left column pairs, serpentine, skipping the timing column. --- */
  let bit = 0;
  const total = codewords.length * 8;
  let upward = true;
  for (let col = size - 1; col > 0; col -= 2) {
    if (col === 6) col = 5; // The timing column is stepped over whole.
    for (let step = 0; step < size; step++) {
      const row = upward ? size - 1 - step : step;
      for (const c of [col, col - 1]) {
        const at = row * size + c;
        if (fixed[at]) continue;
        let v = 0;
        if (bit < total) v = (codewords[bit >> 3] >> (7 - (bit & 7))) & 1;
        bit++;
        // Mask 0.
        if ((row + c) % 2 === 0) v ^= 1;
        dark[at] = v;
      }
    }
    upward = !upward;
  }

  /* --- One SVG path, a unit square per dark module. --- */
  let path = "";
  for (let r = 0; r < size; r++) {
    for (let c = 0; c < size; c++) {
      if (dark[r * size + c]) path += `M${c} ${r}h1v1h-1z`;
    }
  }
  return { size, path };
}
