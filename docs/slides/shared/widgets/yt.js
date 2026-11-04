// A YouTube video as its thumbnail with a play button. The iframe is created on click, so nothing loads or plays
// before, and it is removed when the slide is left. In print: the thumbnail and the link.
// config: { id: 'xyz', start: 0 (seconds), end: 0 (seconds; 0 = to the end), width: 880, height: 495, title: 'caption on the thumbnail', small: false,
//           autoplay: false (true: plays muted and looping whenever its slide is shown, no click needed) }
import { C, h, isPrint } from './util.js';

export function mount(el, cfg) {
  const W = cfg.width || 880, H = cfg.height || 495, start = cfg.start || 0, end = cfg.end || 0;
  const url = `https://www.youtube.com/watch?v=${cfg.id}` + (start ? `&t=${start}s` : '');
  const box = h('div', { style: `position:relative; width:${W}px; height:${H}px; background:#0b0c0f url(https://img.youtube.com/vi/${cfg.id}/hqdefault.jpg) center/cover no-repeat; border:1px solid ${C.line}; border-radius:10px; overflow:hidden; cursor:pointer` });
  const fs = cfg.small ? 12 : 14;
  const caption = (text) => h('div', { style: `position:absolute; left:0; right:0; bottom:0; padding:0.35em 0.6em; background:rgba(0,0,0,0.7); color:#fff; font-size:${fs}px; line-height:1.3; text-align:left` }, text);
  const placeholder = () => {
    box.replaceChildren(
      h('div', { style: 'position:absolute; inset:0; display:flex; align-items:center; justify-content:center' },
        isPrint() ? '' : h('div', { style: `width:${cfg.small ? 54 : 72}px; height:${cfg.small ? 38 : 50}px; background:rgba(0,0,0,0.75); border-radius:12px; display:flex; align-items:center; justify-content:center; color:#fff; font-size:${cfg.small ? 18 : 26}px` }, '▶')),
      caption(isPrint() ? `${cfg.title ? cfg.title + ' · ' : ''}${url}` : (cfg.title || '')));
  };
  const start_ = () => {
    if (isPrint() || box.querySelector('iframe')) return;
    const auto = cfg.autoplay ? `&mute=1&loop=1&playlist=${cfg.id}&controls=0&modestbranding=1` : '';
    const src = `https://www.youtube.com/embed/${cfg.id}?start=${start}${end ? `&end=${end}` : ''}&autoplay=1&rel=0&playsinline=1${auto}`;
    box.replaceChildren(h('iframe', { src, title: cfg.title || 'video', style: 'width:100%; height:100%; border:0', allow: 'autoplay; encrypted-media; picture-in-picture; fullscreen', allowfullscreen: '', referrerpolicy: 'strict-origin-when-cross-origin' }));
  };
  box.addEventListener('click', start_);
  el.classList.add('widget');
  el.style.width = W + 'px';
  el.append(box);
  placeholder();
  if (isPrint()) return;
  const deck = el.closest('.reveal');
  deck?.addEventListener('slidechanged', ev => { if (box.isConnected && !ev.currentSlide?.contains(el) && box.querySelector('iframe')) placeholder(); });
  if (cfg.autoplay) {  // start whenever the slide is shown (the iframe is removed again when it is left, above)
    const shown = () => el.closest('section')?.classList.contains('present') !== false;
    deck?.addEventListener('slidechanged', () => { if (box.isConnected && shown()) start_(); });
    setTimeout(() => { if (shown()) start_(); }, 300);
  }
}
