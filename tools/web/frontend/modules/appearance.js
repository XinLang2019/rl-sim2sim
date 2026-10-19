// Runs synchronously before CSS is loaded, using the existing same-origin script entry.
(() => {
  'use strict';
  const media = matchMedia('(prefers-color-scheme: dark)');
  const valid = choice => ['light', 'dark', 'system'].includes(choice);
  const resolve = choice => choice === 'system' ? (media.matches ? 'dark' : 'light') : choice;
  let choice = 'system';
  try {
    const saved = JSON.parse(localStorage.getItem('rlsim.workbench.v2') || '{}').theme;
    if (valid(saved)) choice = saved;
  } catch { /* Private or corrupt storage uses the system preference. */ }
  document.documentElement.dataset.theme = resolve(choice);
  window.RLsimAppearance = {media, resolve, valid, initialTheme: document.documentElement.dataset.theme, initializedAt: performance.now(), beforeBody: document.body === null};
})();
