// build.mjs inlines .png imports as data: URLs.
declare module '*.png' {
  const src: string;
  export default src;
}
