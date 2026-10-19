// Bounded shared samples, freshness and reset segments; no synthetic UI data.
(()=>{"use strict";const W=window.Workbench;
class TelemetryStore{constructor({notify=true}={}){this.max=1800;this.frames=[];this.descriptors=[];this.epoch=0;this.lastPacket=null;this.lastSim=null;this.lastWall=null;this.lastReset=null;this.lastRevision=null;this.metrics={};this.hasGap=false;this.configure(null,notify)}reset(){this.frames=[];this.lastPacket=null;this.lastSim=null;this.lastWall=null;this.lastReset=null;this.lastRevision=null;this.epoch++;this.metrics={};this.hasGap=false;W.state.mode!=='replay'&&W.charts?.follow()}gap(){this.hasGap=true;W.charts?.invalidate()}configure(resolved,notify=true){
 const d=[],add=(id,label,unit,group,frame,path=[],extra={})=>d.push({id,label,unit,group,frame,path,...extra});
 for(const [k,t] of [['x','位置 X'],['y','位置 Y'],['z','机身高度']])add('body.'+k,t,'m','机体状态','世界坐标系',['位置']);
 for(const [k,t] of [['roll','横滚 Roll'],['pitch','俯仰 Pitch'],['yaw','偏航 Yaw']])add('body.'+k,t,'rad','机体状态','姿态 RPY，世界参考',['姿态']);
 for(const [k,t] of [['vx','线速度 X'],['vy','线速度 Y'],['vz','线速度 Z']])add('body.'+k,t,'m/s','机体状态','世界坐标系',['线速度']);
 for(const [k,t] of [['wx','角速度 X'],['wy','角速度 Y'],['wz','角速度 Z']])add('body.'+k,t,'rad/s','机体状态','世界坐标系',['角速度']);
 const names=resolved?.policy?.joints||resolved?.robot?.joints||[];
 for(const name of names){
  const match=name.match(/^(FL|FR|RL|RR|HL|HR)(?=[_.-]|$)/i),prefix=match?.[1].toUpperCase(),leg=({HL:'RL',HR:'RR'}[prefix]||prefix||'other');
  const legLabel=({FL:'FL · 左前腿',FR:'FR · 右前腿',RL:'RL · 左后腿',RR:'RR · 右后腿'}[leg]||'其他关节');
  const jointLabel=name.replace(/_joint$/,'').replace(/^(FL|FR|RL|RR|HL|HR)[_.-]/i,'');
  for(const [key,label,unit,quantity] of [['q','实际位置','rad','位置'],['target_q','目标位置','rad','位置'],['dq','实际速度','rad/s','速度'],['target_dq','目标速度','rad/s','速度'],['effort','估计力矩','Nm','力矩'],['tau_ff','前馈力矩','Nm','力矩']]){
   add(`joint.${name}.${key}`,`${name} · ${label}`,unit,'关节状态','关节坐标系',[legLabel,quantity],{joint:name,leg,shortLabel:`${jointLabel} · ${label}`});
  }
 }
 if(resolved?.policy?.actions?.count)for(let i=0;i<resolved.policy.actions.count;i++){
  const bindings=(resolved.policy.actions.channels||[]).filter(c=>c.action===i).map(c=>names[c.joint]).filter(Boolean);
  add('policy.action.'+i,'action'+i,'','策略诊断','模型输出'+(bindings.length?' → '+[...new Set(bindings)].join('、'):''),['action']);
 }
 add('performance.inference','推理与解码','ms','运行性能','墙钟耗时',['推理耗时']);
 add('performance.rtf','RTF · 时钟差分估计','×','运行性能','仿真时间 / 接收时间',['仿真效率']);
 add('performance.policy_hz','策略频率 · 实测','Hz','运行性能','策略步数差分 / 接收时间',['策略频率']);
 add('command.vx','网页目标 vx','m/s','控制指令','机体系；浏览器输入，不是后端确认',['平移速度']);
 add('command.vy','网页目标 vy','m/s','控制指令','机体系；浏览器输入，不是后端确认',['平移速度']);
 add('command.wz','网页目标 wz','rad/s','控制指令','机体系；浏览器输入，不是后端确认',['转向速度']);
 this.descriptors=d;if(notify)W.charts?.signalsChanged();}

 ingest(status){const p=status.telemetry;if(!status.bridge_live||!p)return this.gap();const packet=p.service_received_at??p.wall_time;if(packet===this.lastPacket)return;this.lastPacket=packet;const signals=p.signals||{},now=Date.now()/1000,ok=name=>signals[name]&&now-signals[name].received_at<2;const t=ok('clock')?signals.clock.data:null;if(!W.finite(t))return this.gap();const wall=p.service_received_at??p.wall_time??now;const resetCount=ok('controller')?signals.controller.data.reset_count:null,revision=ok('controller')?signals.controller.data.controller_revision:null;const reset=(this.lastSim!==null&&t<this.lastSim-.001)||(W.finite(resetCount)&&W.finite(this.lastReset)&&resetCount!==this.lastReset)||(W.finite(revision)&&W.finite(this.lastRevision)&&revision!==this.lastRevision),gap=this.hasGap||(this.lastWall!==null&&wall-this.lastWall>1.25);if(!reset&&!gap&&this.lastWall!==null&&wall-this.lastWall<1/(this.sampleHz||W.prefs.sampleHz)-.001)return;if(reset||gap){this.epoch++;W.state.mode!=='replay'&&W.charts?.follow();}const values={};const m=ok('model')?signals.model.data:null,c=ok('controller')?signals.controller.data:null;if(m){['x','y','z'].forEach((k,i)=>values['body.'+k]=m.position?.[i]);['vx','vy','vz'].forEach((k,i)=>values['body.'+k]=m.linear_velocity?.[i]);['wx','wy','wz'].forEach((k,i)=>values['body.'+k]=m.angular_velocity?.[i]);const [w,x,y,z]=m.quaternion||[];if([w,x,y,z].every(W.finite)){values['body.roll']=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));values['body.pitch']=Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))));values['body.yaw']=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));}}
 if(c){for(const [i,name] of (c.joint_names||[]).entries())for(const key of ['q','target_q','dq','target_dq','effort','tau_ff'])values[`joint.${name}.${key}`]=c[key]?.[i];for(const [i,a]of(c.actions||[]).entries())values['policy.action.'+i]=a;values['performance.inference']=c.inference_ms;}
 const dt=wall-this.lastWall;if(!reset&&!gap&&this.lastWall!==null&&dt>0){values['performance.rtf']=Math.max(0,(t-this.lastSim)/dt);const prev=this.frames.at(-1);if(c&&prev?.steps!=null&&c.policy_steps>=prev.steps)values['performance.policy_hz']=(c.policy_steps-prev.steps)/dt;}
 if(W.canSustainVelocity()&&W.joystick){['vx','vy','wz'].forEach((k,i)=>values['command.'+k]=W.joystick.target[i])}for(const k of Object.keys(values))if(!W.finite(values[k]))values[k]=null;this.frames.push({t,wall,resetCount,epoch:this.epoch,values,steps:c?.policy_steps??null,messageTime:signals.joints?.message_time??null});if(this.frames.length>this.max)this.frames.splice(0,this.frames.length-this.max);this.lastSim=t;this.lastWall=wall;this.lastReset=resetCount;this.lastRevision=revision;this.hasGap=false;this.metrics={rtf:values['performance.rtf'],policyHz:values['performance.policy_hz']};if(W.state.mode!=='replay')W.$('buffer-info').textContent=`页面缓冲 ${this.frames.length} / ${this.max}`;W.charts?.invalidate();}
 currentEpoch(){return this.frames.filter(f=>f.epoch===this.epoch)}nearest(t){const a=this.currentEpoch();let n=null;for(const f of a)if(!n||Math.abs(f.t-t)<Math.abs(n.t-t))n=f;return n&&Math.abs(n.t-t)<=1.2?n:null}descriptor(id){return this.descriptors.find(d=>d.id===id)}}
W.signalMatches=(d,query)=>{const text=[d.id,d.label,d.unit,d.group,d.frame,...(d.path||[])].join(' ').toLowerCase();return query.trim().toLowerCase().split(/\s+/).every(word=>text.includes(word))};
W.chartData=()=>W.state.mode==='replay'&&W.recordings?.store?W.recordings.store:W.telemetry;
W.telemetry=new TelemetryStore();W.TelemetryStore=TelemetryStore;
})();
