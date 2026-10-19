// Controller lifecycle shares the existing session, bridge, world and camera.
(()=>{'use strict';const W=Workbench,{$,state}=W;
 class Controller{
  constructor(){this.busy=false;$('controller-toggle').onclick=()=>this.toggle().catch(e=>W.notice(e.message))}
  status(){
   const entry=state.current?.telemetry?.signals?.controller,c=entry?.data||{},ack=state.current?.controller;
   return{supported:!!(state.boot?.capabilities?.controller_reload&&c.controller_lifecycle_supported),
    running:ack&&ack.received_at>entry.received_at?ack.running:c.controller_running!==false,
    unknown:!!state.current?.controller_unknown};
  }
  canEdit(){const s=this.status();return !!(state.mode==='live'&&state.activeId===state.selected&&W.live()&&s.supported&&!s.running&&!s.unknown&&!this.busy)}
  render(){
   const s=this.status(),button=$('controller-toggle');
   button.textContent=this.busy?'控制器处理中…':s.running?'停止控制器':'启动控制器';
   button.classList.toggle('controller-stopped',!s.running);
   button.disabled=this.busy||!W.live()||state.mode!=='live'||!s.supported||s.unknown||!!W.resources?.applying;
   button.title=s.unknown?'控制器应用状态未知，请结束当前任务':!s.supported?'当前节点未接入控制器独立启停，构建后需重新启动一次任务':s.running?'停止策略与状态机，保留阻尼；Gazebo 继续运行':'以零指令恢复控制器，再点击站立、启用策略';
  }
  async toggle(){
   const s=this.status();if(this.busy||!W.live()||!s.supported||s.unknown||state.mode!=='live')return;
   if(!s.running&&W.resources.status!=='已应用'){W.notice('有尚未应用的修改，请先应用或撤销编辑。');return}
   this.busy=true;this.render();W.resources.lock();
   try{
    await W.joystick.zero();
    const result=await W.api(`/api/runs/${encodeURIComponent(state.selected)}/controller`,{action:s.running?'stop':'start'},70000);
    state.pending=null;state.current.controller={running:result.running,revision:result.revision,received_at:Date.now()/1000};
    W.notice(s.running?'控制器已停止，可修改策略与参数；仿真继续运行。':'控制器已启动，请站立后启用策略。',true);
    await W.service.poll();
   }finally{this.busy=false;this.render();W.resources.lock();W.renderStatus()}
  }
  async apply(configuration){
   const id=state.selected;
   const result=await W.api(`/api/runs/${encodeURIComponent(id)}/controller`,{action:'apply',configuration},90000);
   if(id!==state.selected)throw new Error('运行会话已改变，请刷新状态');
   state.resolved=result.resolved;
   state.pending=null;state.current.controller={running:result.running,revision:result.revision,received_at:Date.now()/1000};
   state.detail={...state.detail,resolved:result.resolved,terrain:result.terrain};
   W.resources.loaded(state.detail);
   W.telemetry.gap();W.telemetry.configure(result.resolved);
   W.notice('策略与参数已应用到当前控制器，仿真未重启；点击启动控制器后测试。',true);
   await W.service.poll();return result;
  }
 }
 W.controller=new Controller();
})();
