// Pointer navigation owns only view state; it never changes acquisition or control.
(()=>{'use strict';const W=Workbench;
class ChartInteraction{
 constructor(workspace){
  this.workspace=workspace;this.ranges=new Map();this.original=null;this.drag=null;
  const options={signal:workspace.disposer.signal};
  window.addEventListener('blur',()=>this.cancel(),options);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.cancel()},options);
  document.addEventListener('keydown',e=>{if(e.key==='Escape'&&this.drag){this.cancel();e.preventDefault()}},options);
 }
 sync(){
  const source=W.chartData();
  if(this.source!==source||this.epoch!==source.epoch){this.clear();this.source=source;this.epoch=source.epoch}
 }
 clear(){this.cancel();this.ranges.clear();this.original=null;this.update()}
 reset(){
  const c=this.workspace,original=this.original;this.clear();
  if(original){c.span=original.span;c.end=original.end;c.paused=original.paused;c.frozen=original.frozen}
  c.cursor=null;c.windowLabel();c.syncDisplayControls();c.invalidate();
 }
 fitFrames(){return W.state.mode==='replay'?W.recordings.store.currentEpoch():this.workspace.frames()}
 fit(id){
  this.sync();this.cancel();
  const c=this.workspace,pane=c.cards.get(id),frames=this.fitFrames();
  if(!pane||!frames.length||!pane.config.signals.some(id=>W.chartData().descriptor(id)))return;
  if(W.state.mode==='replay'){
   // Explicitly reveal the full selected segment, including unplayed samples.
   W.recordings.pausePlayback();W.recordings.playhead=null;
   if(c.paused)c.frozen=frames.slice();
   W.$('record-position').max=frames.length-1;W.$('record-position').value=frames.length-1;
   W.recordings.position();W.recordings.playbackControls();
  }
  this.ranges.delete(id);this.original=null;
  const start=frames[0].t,end=frames.at(-1).t;
  c.span=end>start?end-start:1;c.end=end>start?end:end+.5;c.cursor=null;
  if(W.state.mode!=='replay')c.pauseDisplay();else c.syncDisplayControls();
  c.windowLabel();c.invalidate();
 }
 update(){this.workspace.invalidate()}
 cancel(){
  const d=this.drag;if(!d)return;this.drag=null;d.selection.hidden=true;
  if(d.canvas.hasPointerCapture?.(d.pointer))d.canvas.releasePointerCapture(d.pointer);
  this.workspace.invalidate();
 }
 point(e,canvas,view){
  const r=canvas.getBoundingClientRect();
  return{x:Math.max(view.left,Math.min(view.right,e.clientX-r.left)),y:Math.max(view.top,Math.min(view.bottom,e.clientY-r.top))};
 }
 displayBounds(){return this.drag?.box?{start:this.drag.view.start,end:this.drag.view.end}:null}
 range(id,unit){
  if(this.drag?.box&&this.drag.id===id)return this.drag.view.range;
  const range=this.ranges.get(id);if(range&&range.unit!==unit){this.ranges.delete(id);return null}return range;
 }
 attach(c){
  const workspace=this.workspace,{canvas,config,selection}=c;
  canvas.title='拖动平移；Shift + 拖动框选放大；双击恢复视图';
  canvas.onwheel=e=>{
   e.preventDefault();this.sync();this.cancel();
   if(W.state.mode==='replay')W.recordings.pausePlayback();
   const b=workspace.bounds(),frames=workspace.frames();
   workspace.span=Math.max(.001,Math.min(W.state.mode==='replay'?Math.max(120,(frames.at(-1)?.t??0)-(frames[0]?.t??0))*2:120,workspace.span*(e.deltaY>0?1.2:.8)));
   workspace.end=b.end;workspace.pauseDisplay();workspace.windowLabel();W.prefs.window=workspace.span;workspace.persist();workspace.invalidate();
  };
  canvas.onpointerdown=e=>{
   if(e.button!==0||this.drag)return;
   this.sync();workspace.draw();const view=c.view;if(!view||config.kind==='value')return;
   const r=canvas.getBoundingClientRect(),x=e.clientX-r.left,y=e.clientY-r.top;
   if(x<view.left||x>view.right||y<view.top||y>view.bottom)return;
   e.preventDefault();canvas.setPointerCapture(e.pointerId);
   const start=this.point(e,canvas,view);
   this.drag={id:config.id,canvas,selection,pointer:e.pointerId,view:{...view},start,end:start,box:!!e.shiftKey,original:{span:workspace.span,end:workspace.end,paused:workspace.paused,frozen:workspace.frozen}};
   if(this.drag.box){workspace.cursor=null;c.tooltip.hidden=true;this.rectangle(this.drag)}
  };
  canvas.onpointermove=e=>{
   const d=this.drag;
   if(d){
    if(d.canvas!==canvas||d.pointer!==e.pointerId)return;
    if(!this.valid(d))return;
    d.end=this.point(e,canvas,d.view);
    if(d.box)this.rectangle(d);
    else{
     if(!d.panning&&Math.abs(d.end.x-d.start.x)<2)return;
     d.panning=true;workspace.pauseDisplay();
     workspace.end=d.view.end-(d.end.x-d.start.x)/(d.view.right-d.view.left)*(d.view.end-d.view.start);
     const a=workspace.frames();if(a.length)workspace.end=Math.max(a[0].t,Math.min(a.at(-1).t,workspace.end));
    }
   }else{
    const view=c.view;if(!view)return;const p=this.point(e,canvas,view);
    workspace.cursor=view.start+(p.x-view.left)/(view.right-view.left)*(view.end-view.start);
   }
   workspace.invalidate();
  };
  canvas.onpointerup=e=>{
   const d=this.drag;if(!d||d.canvas!==canvas||d.pointer!==e.pointerId)return;
   if(!this.valid(d))return;
   d.end=this.point(e,canvas,d.view);this.cancel();
   if(!d.box||Math.abs(d.end.x-d.start.x)<6||Math.abs(d.end.y-d.start.y)<6)return;
   const v=d.view,time=x=>v.start+(x-v.left)/(v.right-v.left)*(v.end-v.start),value=y=>v.range.max-(y-v.top)/(v.bottom-v.top)*(v.range.max-v.range.min);
   const start=time(Math.min(d.start.x,d.end.x)),end=time(Math.max(d.start.x,d.end.x));
   const min=value(Math.max(d.start.y,d.end.y)),max=value(Math.min(d.start.y,d.end.y));
   if(![start,end,min,max].every(Number.isFinite)||end<=start||max<=min)return;
   this.original=this.original||d.original;workspace.span=end-start;workspace.end=end;workspace.cursor=null;
   // Store canonical units so switching rad/deg does not change the selected area.
   this.ranges.set(config.id,{min,max,unit:v.unit});workspace.pauseDisplay();workspace.windowLabel();this.update();
  };
  canvas.onpointercancel=canvas.onlostpointercapture=e=>{if(this.drag?.canvas===canvas&&(!e||e.pointerId===this.drag.pointer))this.cancel()};
  canvas.onpointerleave=()=>{if(!this.drag){workspace.cursor=null;workspace.invalidate()}};
  canvas.ondblclick=e=>{e.preventDefault();this.cancel();this.reset()};
 }
 valid(d){
  // Geometry or data source may have changed while the pointer was captured.
  this.sync();if(this.drag!==d)return false;
  if(Math.abs(d.canvas.clientWidth-10-d.view.right)>1||Math.abs(d.canvas.clientHeight-23-d.view.bottom)>1){this.cancel();return false}
  return true;
 }
 rectangle(d){
  const left=Math.min(d.start.x,d.end.x),top=Math.min(d.start.y,d.end.y);
  Object.assign(d.selection.style,{left:left+'px',top:top+'px',width:Math.abs(d.end.x-d.start.x)+'px',height:Math.abs(d.end.y-d.start.y)+'px'});
  d.selection.hidden=false;
 }
}
W.ChartInteraction=ChartInteraction;
})();
