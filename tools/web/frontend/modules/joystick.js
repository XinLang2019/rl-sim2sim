// Exclusive web input sources and an ordered velocity lease; ROS watchdog remains authoritative.
(()=>{"use strict";
const W=window.Workbench,{$}=W,axes=['vx','vy','wz'];
class VelocityLease{constructor(send,onTarget=()=>{}){this.send=send;this.onTarget=onTarget;this.target=[0,0,0];this.tail=Promise.resolve();this.generation=0;this.timer=null;this.runId=''}enqueue(values,gen,force=false){const id=this.runId;this.tail=this.tail.catch(()=>{}).then(()=>{if(!force&&gen!==this.generation)return;return this.send(id,values)});return this.tail}begin(id,values){this.runId=id;this.generation++;const gen=this.generation;clearInterval(this.timer);this.target=values.slice();this.onTarget(this.target);this.enqueue(this.target,gen).catch(()=>{});this.timer=setInterval(()=>this.enqueue(this.target.slice(),gen).catch(()=>{}),200)}zero(){clearInterval(this.timer);this.timer=null;this.generation++;this.target=[0,0,0];this.onTarget(this.target);if(this.runId){const task=this.enqueue([0,0,0],this.generation,true);this.runId='';return task}return Promise.resolve()}}

class Joystick {
 constructor(){
  this.enabled=false;this.focused=document.hasFocus();this.keys=new Set();this.source=null;
  this.pointers=new Map();this.sticks={left:[0,0],right:[0,0]};this.stickValues={left:[0,0],right:[0,0]};this.latched=false;this.target=[0,0,0];this.preserveInputs=false;
  this.bounds=[[-1,1],[-.5,.5],[-1.57,1.57]];
  this.fastBounds=[-2,2];this.fastMode=false;
  this.lease=new VelocityLease((id,v)=>this.send(id,v),v=>{
   this.target=v;
   if(!this.preserveInputs)axes.forEach((k,i)=>$(k).value=Number(v[i].toFixed(3)));
   this.paint();
  });
  this.events();this.updateBounds();
 }
 paint(){
  const moving=this.target.some(v=>v!==0);
  $('drive-state').textContent=this.latched&&moving?'恒速保持中':!this.enabled?'等待策略':!moving?'已启用 · 零输入':'网页输入中';
  for(const [side,pad] of [['left',$('joystick-pad')],['right',$('joystick-right-pad')]])pad.classList.toggle('holding',this.latched&&this.sticks[side].some(v=>Math.abs(v)>.08));
  $('joystick-right-pad').classList.toggle('fast',this.fastMode);
  $('drive-mode').textContent=this.fastMode?'高速':'常速';$('drive-mode').classList.toggle('fast',this.fastMode);
  [$('vx').min,$('vx').max]=this.fastMode?this.fastBounds:this.bounds[0];
  const clear=$('data-velocity-zero');
  clear.hidden=!(W.state.page==='data'&&this.latched&&moving);
  clear.title=`当前保持 vx ${this.target[0]} m/s、vy ${this.target[1]} m/s、yaw ${this.target[2]} rad/s；点击发送零速度`;
 }
 async send(id,values){
  try{
   await W.api(`/api/runs/${encodeURIComponent(id)}/command`,{type:'velocity',value:values},2500);
   $('velocity-ack').hidden=false;$('velocity-ack').textContent='指令已入队';
  }catch(e){this.disable();$('velocity-ack').hidden=false;$('velocity-ack').textContent='指令发送失败：'+e.message;throw e}
 }
 limits(){return this.bounds.map(pair=>pair.slice())}
 clamp(values,fast=this.fastMode){return values.map((v,i)=>{const range=i===0&&fast?this.fastBounds:this.bounds[i];return Math.max(range[0],Math.min(range[1],v))})}
 updateBounds(){
  axes.forEach((k,i)=>{
   $(k).min=this.bounds[i][0];$(k).max=this.bounds[i][1];
   for(const [j,end] of ['min','max'].entries())$('limit-'+k+'-'+end).value=this.bounds[i][j];
  });
  ['min','max'].forEach((end,i)=>$('limit-fast-vx-'+end).value=this.fastBounds[i]);
 }
 editBounds(){
  const caps=[2,1.5,3],next=axes.map(k=>['min','max'].map(end=>$('limit-'+k+'-'+end).valueAsNumber));
  const fast=['min','max'].map(end=>$('limit-fast-vx-'+end).valueAsNumber);
  if([...next,fast].some(([lo,hi],i)=>!Number.isFinite(lo)||!Number.isFinite(hi)||lo>0||hi<0||lo>=hi||lo < -(caps[i]||2)||hi>(caps[i]||2))){
   this.updateBounds();this.paint();W.feedback('velocity-limit-error','限幅需包含零且下限小于上限；后端允许 vx ±2、vy ±1.5、yaw ±3。');return;
  }
  this.bounds=next;this.fastBounds=fast;this.updateBounds();this.paint();W.feedback('velocity-limit-error','');
  const clip=(v,r)=>Math.max(r[0],Math.min(r[1],v));
  this.stickValues.left=this.stickValues.left.map((v,i)=>clip(v,this.bounds[i]));
  this.stickValues.right=this.stickValues.right.map((v,i)=>clip(v,i?this.bounds[2]:this.fastBounds));
  // Shrinking a range takes effect immediately, including an existing held command.
  if(this.lease.timer){const values=this.clamp(this.target);this.lease.begin(W.state.selected,values)}
 }
 begin(source,normalized,raw=false,fast=false){
  if(!this.enabled||!W.canDrive()||!this.focused||document.hidden)return false;
  const priority={keyboard:1,typed:2,pointer:3};
  if(this.pointers.size&&source!=='pointer')return false;
  if(!this.latched&&this.source&&priority[this.source]>priority[source])return false;
  if(normalized.some(v=>!Number.isFinite(v)))return false;
  if(source!=='pointer')this.clearSticks();
  this.latched=false;this.source=source;this.fastMode=fast;
  const values=raw?this.clamp(normalized,fast):normalized.map((v,i)=>this.mapAxis(v,i===0&&fast?this.fastBounds:this.bounds[i]));
  this.lease.begin(W.state.selected,values);return true;
 }
 mapAxis(v,range){const dead=.08,x=Math.abs(v)<=dead?0:Math.sign(v)*(Math.min(1,Math.abs(v))-dead)/(1-dead);return x*(x>=0?range[1]:-range[0])}
 clearSticks(){
  const pointers=[...this.pointers.values()];this.pointers.clear();
  for(const {element,id} of pointers)if(element.hasPointerCapture(id))element.releasePointerCapture(id);
  this.sticks={left:[0,0],right:[0,0]};this.stickValues={left:[0,0],right:[0,0]};
  for(const side of ['','right-']){$('joystick-'+side+'thumb').style.transform='';$('joystick-'+side+'pad').classList.remove('active')}
 }
 applySticks(){
  const [left,right]=[this.stickValues.left,this.stickValues.right],fast=Math.abs(this.sticks.right[0])>.08;
  const values=[fast?right[0]:left[0],left[1],right[1]];
  if(!this.begin('pointer',values,true,fast))return;
  if(!this.pointers.size){if(values.some(Boolean)){this.latched=true;this.source=null;this.paint()}else this.zero()}
 }
 release(source){
  if(this.source!==source)return;
  if($('velocity-hold').checked&&this.enabled&&W.canDrive()&&this.focused&&!document.hidden&&this.target.some(Boolean)){
   this.latched=true;this.source=null;this.paint();
  }else this.zero();
 }
 async constant(){
  const values=axes.map(k=>$(k).valueAsNumber);
  if(values.some(v=>!Number.isFinite(v))){W.notice('请输入有效的 vx、vy 和 yaw 速度。');return}
  if(!this.enabled||!W.canDrive())return;
  // An explicit send replaces any earlier source; editing alone never sends motion.
  const fast=this.fastMode;this.clearSticks();this.keys.clear();this.source=null;
  $('joystick').focus();
  if(!this.begin('typed',values,true,fast))return;
  if(this.target.every(v=>v===0)){await this.zero();return}
  this.latched=true;this.source=null;this.paint();
  await this.lease.tail;
 }
 zero({preserveInputs=false}={}){
  this.source=null;this.latched=false;this.fastMode=false;this.clearSticks();this.keys.clear();
  this.preserveInputs=preserveInputs;const task=this.lease.zero();this.preserveInputs=false;
  return task.catch(()=>{});
 }
 disable(){this.enabled=false;this.zero();this.paint()}
 refresh(){
  // The session owns an explicit held command. Only fresh joystick input is
  // scoped to the simulation page; viewing live charts must not revoke its lease.
  const ready=!!(W.canSustainVelocity()&&this.focused&&!document.hidden&&(!this.lease.timer||this.lease.runId===W.state.selected));
  const allowed=ready&&W.state.page==='simulation';
  if(!ready||(!allowed&&!this.latched)){
   if(this.enabled||this.lease.timer||this.target.some(v=>v!==0))this.disable();
  }else{
   this.enabled=allowed;
   if(!allowed){this.keys.clear();this.source=null}
  }
  this.paint();
  $('drive').disabled=!allowed;
  for(const id of ['joystick-pad','joystick-right-pad'])$(id).setAttribute('aria-disabled',String(!allowed));
 }
 events(){
  const region=$('joystick');
  for(const [side,id,thumbId] of [['left','joystick-pad','joystick-thumb'],['right','joystick-right-pad','joystick-right-thumb']]){
   const pad=$(id),thumb=$(thumbId);
   const move=e=>{
    if(!this.enabled||!W.canDrive())return;
    const r=pad.getBoundingClientRect(),x=(e.clientX-r.left-r.width/2)/(r.width*.38),y=(e.clientY-r.top-r.height/2)/(r.height*.38),norm=Math.max(1,Math.hypot(x,y)),nx=x/norm,ny=y/norm;
    this.sticks[side]=[-ny,-nx];
    this.stickValues[side]=this.sticks[side].map((v,i)=>this.mapAxis(v,side==='left'?this.bounds[i]:i?this.bounds[2]:this.fastBounds));
    thumb.style.transform=`translate(${nx*r.width*.32}px,${ny*r.height*.32}px)`;pad.classList.add('active');this.applySticks();
   };
   pad.onpointerdown=e=>{
    if(!this.enabled||!W.canDrive()||this.pointers.has(side)||[...this.pointers.values()].some(p=>p.id===e.pointerId))return;
    e.preventDefault();region.focus();this.keys.clear();
    this.pointers.set(side,{id:e.pointerId,element:pad});pad.setPointerCapture(e.pointerId);move(e);
   };
   pad.onpointermove=e=>{if(this.pointers.get(side)?.id===e.pointerId)move(e)};
   pad.addEventListener('pointerup',e=>{
    if(this.pointers.get(side)?.id!==e.pointerId)return;
    this.pointers.delete(side);
    if(pad.hasPointerCapture(e.pointerId))pad.releasePointerCapture(e.pointerId);
    if(!$('velocity-hold').checked){this.sticks[side]=[0,0];this.stickValues[side]=[0,0];thumb.style.transform='';pad.classList.remove('active')}
    this.applySticks();
   });
   pad.addEventListener('lostpointercapture',e=>{if(this.pointers.get(side)?.id===e.pointerId)this.zero()});
   pad.addEventListener('pointercancel',()=>this.zero());
  }
  $('drive').onclick=()=>W.busy($('drive'),()=>this.constant()).catch(()=>{});
  $('velocity-hold').onchange=()=>{if(!$('velocity-hold').checked)this.zero();this.paint()};
  for(const k of [...axes,'fast-vx'])for(const end of ['min','max'])$('limit-'+k+'-'+end).onchange=()=>this.editBounds();
  const editable=t=>t.closest('input,select,textarea,[contenteditable=true]');
  const keySteps={KeyW:[0,.1],KeyS:[0,-.1],KeyA:[1,.1],KeyD:[1,-.1],KeyQ:[2,.1],KeyE:[2,-.1]};
  region.onkeydown=e=>{
   if(!this.enabled||editable(e.target)||e.ctrlKey||e.metaKey||e.altKey||!(e.code in keySteps||e.code==='Space'))return;
   if(e.code==='Space'&&e.target.closest('button'))return;
   if(this.pointers.size&&e.code!=='Space')return;
   e.preventDefault();if(e.code==='Space')return this.zero();
   // One physical press changes only its axis. Key release and OS auto-repeat
   // must not replace or accelerate the session's held velocity.
   if(e.repeat||this.keys.has(e.code))return;
   const [axis,step]=keySteps[e.code],values=this.target.slice();
   values[axis]=Math.round((values[axis]+step)*1e9)/1e9;
   if(!this.begin('keyboard',values,true,this.fastMode))return;
   this.keys.add(e.code);
   if(this.target.every(v=>v===0)){this.zero();return}
   this.latched=true;this.source=null;this.paint();
  };
  region.onkeyup=e=>{if(this.keys.delete(e.code))e.preventDefault()};
  region.addEventListener('focusout',e=>{
   const transfer=destination=>{
    // A held command belongs to the running session, not the focused panel.
    // Window blur and loss of control readiness cancel it separately.
    if(this.latched){this.keys.clear();return}
    if(!region.contains(destination))this.zero();
    else if(editable(destination)&&this.lease.timer)this.zero({preserveInputs:true});
   };
   // Chromium runs a microtask checkpoint between blur and focusin. During
   // that checkpoint activeElement is body, even for an internal transfer.
   // relatedTarget describes the intended destination without that transient gap.
   if(e.relatedTarget){transfer(e.relatedTarget);return}
   queueMicrotask(()=>transfer(document.activeElement));
  });
  window.addEventListener('blur',()=>{this.focused=false;this.disable()});
  window.addEventListener('focus',()=>{this.focused=true;this.refresh()});
  window.addEventListener('pagehide',()=>this.disable());
  document.addEventListener('visibilitychange',()=>{if(document.hidden)this.disable();else this.refresh()});
 }
}
W.VelocityLease=VelocityLease;W.joystick=new Joystick();
})();
