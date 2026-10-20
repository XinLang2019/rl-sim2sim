// Service-owned output retention; independent of the simulation and recording.
(() => {
  'use strict';
  const W = window.Workbench, {$, state} = W;
  class OutputSettings {
    constructor() {
      this.value = 5; this.saving = false; this.available = false;
      $('task-output-limit').onchange = () => this.save();
    }
    load() {
      const capabilities = state.boot?.capabilities || {};
      this.available = Array.isArray(capabilities.task_output_limit_options);
      this.value = capabilities.task_output_limit ?? 5;
      $('task-output-limit').value = String(this.value);
      this.render();
    }
    render() {
      $('task-output-limit').disabled = !this.available || !state.online || this.saving;
    }
    async save() {
      if (this.saving || !this.available || !state.online) return;
      const raw = $('task-output-limit').value, value = raw === 'all' ? 'all' : Number(raw);
      this.saving = true; this.render();
      try {
        const result = await W.api('/api/settings/output', {task_output_limit: value});
        this.value = result.task_output_limit;
        state.boot.capabilities.task_output_limit = this.value;
        W.notice(this.value === 'all' ? '已保存：保留所有任务日志。' : `已保存：保留最近 ${this.value} 次任务日志，下次生成任务时生效。`, true);
      } catch (error) {
        W.notice('保留设置保存失败：' + error.message);
      } finally {
        $('task-output-limit').value = String(this.value);
        this.saving = false; this.render();
      }
    }
  }
  W.outputSettings = new OutputSettings();
})();
