// Slido poll placeholder.
// config: {
//   question: "...", options: ["...", ...],
//   event: "<event hash>"   optional; from Slido → Settings → Integrations → Embed Slido
//   code: "#1234567"        optional join code shown to students
// }
// Without `event` the slide shows the question and options (works in print and without internet).
// With `event` a live participant view (iframe, /polls only) is shown next to the question.
import { h, isPrint } from './util.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const letters = 'ABCDEFGH';
  const left = h('div', { style: 'flex:1' },
    h('div', { style: 'display:flex; align-items:center; gap:0.6em; margin-bottom:0.6em' },
      h('span', { style: 'background:#3fb54d; color:#fff; font-weight:700; border-radius:6px; padding:0.1em 0.5em' }, 'slido'),
      cfg.code ? h('span', { class: 'readout' }, `slido.com · ${cfg.code}`) : h('span', { class: 'dim' }, 'poll'),
    ),
    h('div', { style: 'font-size:1.6em; margin-bottom:0.5em' }, cfg.question || ''),
    h('ol', { style: 'list-style:none; margin:0; padding:0; font-size:1.4em; display:grid; grid-template-columns:repeat(2, auto); gap:0.3em 2em; justify-content:start' },
      (cfg.options || []).map((o, i) => h('li', { style: 'margin:0.25em 0' },
        h('b', {}, letters[i] + '  '), o))),
  );
  const row = h('div', { style: 'display:flex; gap:1em; align-items:flex-start' }, left);
  if (cfg.event && !isPrint()) {
    row.append(h('iframe', {
      src: `https://app.sli.do/event/${cfg.event}/polls`,
      style: 'width:420px; height:480px; border:1px solid #3a3f4b; border-radius:8px; background:#fff',
      class: 'interactive-only',
      allow: 'clipboard-write',
    }));
  }
  el.append(row);
  window.renderMathInElement?.(left, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
}
