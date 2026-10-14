// Shared deck bootstrap: reveal.js + KaTeX auto-render + lazy widget mounting.
// Usage in a lecture: <script type="module" src="../../shared/deck.js"></script>
// Widgets: <div data-widget="warp2d" data-config='{"preset":"skew"}'></div>
import Reveal from '../vendor/reveal/reveal.mjs';
import Notes from '../vendor/reveal/plugin/notes.mjs';
import { applyTheme } from './theme.js';

const widgetModules = {
  warp2d: () => import('./widgets/warp2d.js'),
  p3p: () => import('./widgets/p3p.js'),
  pnpnoise: () => import('./widgets/pnpnoise.js'),
  homog: () => import('./widgets/homog.js'),
  imgops: () => import('./widgets/imgops.js'),
  shear3d: () => import('./widgets/shear3d.js'),
  slido: () => import('./widgets/slido.js'),
  rot3d: () => import('./widgets/rot3d.js'),
  camera3d: () => import('./widgets/camera3d.js'),
  distort: () => import('./widgets/distort.js'),
  order2d: () => import('./widgets/order2d.js'),
  fwdbwd: () => import('./widgets/fwdbwd.js'),
  lines2d: () => import('./widgets/lines2d.js'),
  pinhole3d: () => import('./widgets/pinhole3d.js'),
  teaser: () => import('./widgets/teaser.js'),
  rigid3d: () => import('./widgets/rigid3d.js'),
  pinhole2d: () => import('./widgets/pinhole2d.js'),
  lens2d: () => import('./widgets/lens2d.js'),
  calib: () => import('./widgets/calib.js'),
  lagrange: () => import('./widgets/lagrange.js'),
  ransac: () => import('./widgets/ransac.js'),
  pnp: () => import('./widgets/pnp.js'),
  stereo2d: () => import('./widgets/stereo2d.js'),
  stereomatch: () => import('./widgets/stereomatch.js'),
  earthfield: () => import('./widgets/earthfield.js'),
  magsphere: () => import('./widgets/magsphere.js'),
  headingerr: () => import('./widgets/headingerr.js'),
  orientcheck: () => import('./widgets/orientcheck.js'),
  magekf: () => import('./widgets/magekf.js'),
  magplane: () => import('./widgets/magplane.js'),
  softiron2d: () => import('./widgets/softiron2d.js'),
  extrinsics3d: () => import('./widgets/extrinsics3d.js'),
  projsteps: () => import('./widgets/projsteps.js'),
  stereo3d: () => import('./widgets/stereo3d.js'),
  shutter: () => import('./widgets/shutter.js'),
  eyes2d: () => import('./widgets/eyes2d.js'),
};

function renderMath(root) {
  window.renderMathInElement(root, {
    delimiters: [
      { left: '$$', right: '$$', display: true },
      { left: '\\[', right: '\\]', display: true },
      { left: '$', right: '$', display: false },
      { left: '\\(', right: '\\)', display: false },
    ],
    throwOnError: false,
  });
}

async function mountWidget(el) {
  if (el.dataset.mounted) return;
  el.dataset.mounted = '1';
  const gen = el.dataset.gen = String((+el.dataset.gen || 0) + 1);
  const load = widgetModules[el.dataset.widget];
  if (!load) { el.textContent = `Unknown widget: ${el.dataset.widget}`; return; }
  const config = el.dataset.config ? JSON.parse(el.dataset.config) : {};
  const mod = await load();
  if (el.dataset.gen !== gen || !el.dataset.mounted) return; // unmounted while loading
  mod.mount(el, config);
}

// Browsers allow only ~16 live WebGL contexts; release widgets on slides we are not near.
function unmountWidget(el) {
  if (!el.dataset.mounted) return;
  el.querySelectorAll('canvas').forEach(c => {
    const gl = c.getContext('webgl2') || c.getContext('webgl');
    gl?.getExtension('WEBGL_lose_context')?.loseContext();
  });
  el.replaceChildren();
  el.removeAttribute('style');
  el.classList.remove('widget');
  delete el.dataset.mounted;
  el.dataset.gen = String((+el.dataset.gen || 0) + 1);
}

await applyTheme(); // light colours for ?print-pdf and ?light

const deck = new Reveal({
  width: 1280,
  height: 720,
  margin: 0.04,
  hash: true,
  slideNumber: 'c/t',
  transition: 'fade',
  transitionSpeed: 'fast',
  center: false,
  pdfSeparateFragments: false,
  plugins: [Notes],
});

await deck.initialize();
renderMath(document.querySelector('.reveal .slides'));

const frames = n => new Promise(res => { const f = k => (k ? requestAnimationFrame(() => f(k - 1)) : res()); f(n); });

// Print view: replace a rendered WebGL canvas by a still image and free its context, so that decks with
// more 3D figures than the browser's WebGL context limit (~16) still print every one of them.
function freezeWebGL(el) {
  el.querySelectorAll('canvas').forEach(c => {
    const gl = c.getContext('webgl2') || c.getContext('webgl');
    if (!gl) return;
    const img = new Image();
    img.src = c.toDataURL('image/png');
    img.style.cssText = c.style.cssText;
    img.style.width = c.clientWidth + 'px';
    img.style.height = c.clientHeight + 'px';
    img.style.margin = '0';
    c.replaceWith(img);
    gl.getExtension('WEBGL_lose_context')?.loseContext();
  });
}

// Print view: mount every widget, one at a time, then set window.__deckReady (make-pdf.mjs waits for it).
async function mountAllForPrint() {
  for (const el of document.querySelectorAll('[data-widget]')) {
    await mountWidget(el);
    await frames(3);
    freezeWebGL(el);
  }
  await document.fonts.ready;
  renderMath(document.querySelector('.reveal .slides'));
  await frames(3);
  window.__deckReady = true;
}

// Mount widgets on the current slide and its neighbours; in print view mount everything.
let printing = false;
function mountAround() {
  if (deck.isPrintView()) {
    if (!printing) { printing = true; mountAllForPrint(); }
    return;
  }
  const cur = deck.getCurrentSlide();
  const keep = [cur?.previousElementSibling, cur, cur?.nextElementSibling].filter(Boolean);
  document.querySelectorAll('.reveal .slides section').forEach(sec => {
    if (!keep.includes(sec)) sec.querySelectorAll('[data-widget][data-mounted]').forEach(unmountWidget);
  });
  keep.forEach(s => s.querySelectorAll('[data-widget]').forEach(mountWidget));
}

deck.on('ready', mountAround);
deck.on('slidechanged', mountAround);
mountAround();

// Chrome lays out SVG text at the size the figure had at that moment and may keep it when the figure is
// rescaled (reveal resizes the slides, a widget next to it mounts, fonts load): the labels drift away from their
// lines. Toggling an inherited text property forces a fresh text layout.
function relayoutSvgText(svgs) {
  svgs.forEach(s => { s.style.textRendering = 'geometricPrecision'; });
  requestAnimationFrame(() => svgs.forEach(s => { s.style.textRendering = ''; }));
}
const svgResize = new ResizeObserver(entries => relayoutSvgText(entries.map(e => e.target)));
document.querySelectorAll('.reveal .slides svg').forEach(s => svgResize.observe(s));
deck.on('resize', () => relayoutSvgText(document.querySelectorAll('.reveal .slides section.present svg')));
document.fonts.ready.then(() => relayoutSvgText(document.querySelectorAll('.reveal .slides svg')));

window.deck = deck;
