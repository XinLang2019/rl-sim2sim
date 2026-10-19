// Bounded log tails. Clearing the browser view never deletes server logs.
(() => {
  'use strict';
  const W = window.Workbench, {$, state} = W;
  class LogPanel {
    constructor() {
      this.run = ''; this.raw = {}; this.display = {}; this.loading = false;
      $('log-level').onchange = () => this.render();
      $('log-kind').onchange = () => this.render();
      $('log-follow').onchange = () => this.render();
      $('clear-log-view').onclick = () => {
        this.display[$('log-kind').value] = ''; this.render();
      };
    }
    ingest(id, logs) {
      if (id !== this.run) { this.run = id; this.raw = {}; this.display = {}; }
      for (const [kind, value] of Object.entries(logs)) {
        const next = String(value).replace(/\x1b\[[0-9;]*m/g, '').slice(-48000);
        const old = this.raw[kind] || '';
        if (next === old) continue;
        let addition = next;
        if (next.startsWith(old)) addition = next.slice(old.length);
        else if (old) {
          // Longest matching suffix survives rotation of the server's tail window.
          const combined = next + '\0' + old, prefix = new Uint32Array(combined.length);
          for (let i = 1; i < combined.length; i++) {
            let j = prefix[i - 1];
            while (j && combined[i] !== combined[j]) j = prefix[j - 1];
            if (combined[i] === combined[j]) j++;
            prefix[i] = j;
          }
          const overlap = Math.min(prefix[prefix.length - 1], next.length);
          addition = next.slice(overlap);
        }
        this.raw[kind] = next;
        this.display[kind] = ((this.display[kind] || '') + addition).slice(-48000).split('\n').slice(-300).join('\n');
      }
      state.logs = logs; this.render();
    }
    render() {
      const source = this.display[$('log-kind').value] || '', level = $('log-level').value;
      let previous = 'other';
      const lines = source.split('\n').filter(line => {
        const match = line.match(/\b(DEBUG|INFO|WARNING|WARN|ERROR|FATAL)\b/i);
        if (match) previous = {debug:'debug',info:'info',warning:'warn',warn:'warn',error:'error',fatal:'error'}[match[1].toLowerCase()];
        return level === 'all' || previous === level;
      });
      const text = !source ? '暂无显示内容。清空显示不删除服务端日志。' : lines.join('\n') || '没有匹配此级别的日志。';
      if ($('logs').textContent !== text) $('logs').textContent = text;
      $('log-summary').textContent = `显示 ${source ? lines.length : 0} 行 / 上限 300 行 · 未标级别的日志单独筛选 · 仅清空本地显示`;
      if ($('log-follow').checked) $('logs').scrollTop = $('logs').scrollHeight;
    }
    async refresh() {
      if (!state.selected || this.loading) return;
      this.loading = true; const id = state.selected;
      try {
        const logs = await W.api(`/api/runs/${encodeURIComponent(id)}/logs`, undefined, 2500);
        if (id === state.selected) this.ingest(id, logs);
      } catch (error) { $('log-summary').textContent = '日志读取失败：' + error.message; }
      finally { this.loading = false; }
    }
  }
  W.logs = new LogPanel();
})();
