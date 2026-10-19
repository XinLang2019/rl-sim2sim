// Add/edit a monitoring canvas using the current backend's signal descriptors.
(() => {
  'use strict';
  const W = window.Workbench, {$, el} = W;
  class ChartEditor {
    constructor() {
      this.id = null; this.selection = new Set();
      for (const id of ['canvas-group','canvas-joint','canvas-unit','canvas-search']) $(id).oninput = () => this.signals();
      $('canvas-form').onsubmit = event => { event.preventDefault(); this.save(); };
    }
    open(config = null) {
      this.id = config?.id || null;
      this.selection = new Set(config?.signals || W.charts.selected);
      $('canvas-dialog-title').textContent = config ? '编辑监测画布' : '添加监测画布';
      $('canvas-title').value = config?.title || '自定义监测';
      $('canvas-kind').value = config?.kind || 'line';
      $('canvas-angle-unit').value = config?.angleUnit || '';
      $('canvas-search').value = '';
      W.options($('canvas-group'), [['','全部类型'], ...[...new Set(W.chartData().descriptors.map(d => d.group))].map(x => [x,x])]);
      W.options($('canvas-joint'), [['','全部关节 / 机体'], ...[...new Set(W.chartData().descriptors.map(d=>d.joint).filter(Boolean))].map(x => [x,x])]);
      W.options($('canvas-unit'), [['','全部单位'], ...[...new Set(W.chartData().descriptors.map(d => d.unit))].map(x => [x || 'unitless', x || '无量纲'])]);
      this.signals(); $('canvas-dialog').showModal();
    }
    signals() {
      const group = $('canvas-group').value, joint = $('canvas-joint').value, unit = $('canvas-unit').value;
      const query = $('canvas-search').value.toLowerCase();
      const descriptors = W.chartData().descriptors.filter(d => (!group || d.group === group) &&
        (!joint || (d.joint===joint||d.id.startsWith('joint.' + joint + '.'))) &&
        (!unit || (d.unit || 'unitless') === unit) && W.signalMatches(d, query));
      $('canvas-signals').replaceChildren(...descriptors.map(d => {
        const label = el('label'), input = el('input'); input.type = 'checkbox'; input.checked = this.selection.has(d.id);
        input.value = d.id; label.title = d.frame;
        input.onchange = () => { input.checked ? this.selection.add(d.id) : this.selection.delete(d.id); this.count(); };
        label.append(input, el('span','',d.label), el('small','',d.unit || '无量纲')); return label;
      }));
      if (!descriptors.length) $('canvas-signals').append(el('p','hint','此范围没有已接入信号。'));
      this.count();
    }
    count() { $('canvas-selection').textContent = `已选 ${this.selection.size} 条信号 · 单位与坐标系来自实际接口`; }
    save() {
      if (!$('canvas-form').reportValidity()) return;
      const title = $('canvas-title').value.trim();
      if (!title) return W.notice('请输入画布名称。');
      const groups = new Map();
      for (const id of this.selection) {
        const descriptor = W.chartData().descriptor(id);
        if (!descriptor) continue;
        if (!groups.has(descriptor.unit)) groups.set(descriptor.unit, []);
        groups.get(descriptor.unit).push(id);
      }
      if (!groups.size) return W.notice('请选择至少一个当前接口提供的信号。');
      if ([...groups.values()].some(ids => ids.length > 32)) return W.notice('每张画布最多 32 条信号。');
      if (this.id && groups.size > 1) return W.notice('编辑单张画布时请选择相同单位；新增画布可按单位自动拆分。');
      const fields = {kind:$('canvas-kind').value, angleUnit:$('canvas-angle-unit').value};
      if (this.id) {
        const config = W.prefs.charts.find(c => c.id === this.id);
        if (!config) return W.notice('该画布已删除，请重新创建。');
        Object.assign(config, fields, {title, signals:[...groups.values()][0]});
        config.hidden = (config.hidden || []).filter(id => config.signals.includes(id));
      } else {
        for (const [unit, signals] of groups) {const config={id:crypto.randomUUID(), title:groups.size > 1 ? `${title} · ${unit || '无量纲'}` : title, signals, ...fields};W.prefs.charts.push(config);W.charts.layout.add(config);}
      }
      W.charts.persist(); W.charts.rebuild();
      if (!this.id) { W.charts.clearSelection(); this.selection.clear(); }
      $('canvas-dialog').close();
    }
  }
  W.chartEditor = new ChartEditor();
})();
