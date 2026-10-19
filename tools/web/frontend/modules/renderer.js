// Real Gazebo camera adapter. Camera IO never sends robot commands.
(()=>{"use strict";const W=window.Workbench,{$}=W;
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),azimuth=v=>((v+Math.PI)%(2*Math.PI)+2*Math.PI)%(2*Math.PI)-Math.PI;
class GazeboCameraRenderer{
 constructor(){this.kind='gazebo-camera-jpeg';this.connection='idle';this.id='';this.config=null;this.generation=0;this.sequence=-1;this.url=null;this.error='';this.active=false;this.autoConnect=true;this.lastSent=0;this.cameraNextAt=0;this.sending=false;this.fetching=false;this.lastFetch=0;this.displayedFrames=0;this.frameTimes=[];this.frameTimer=null;this.nextFetchAt=0;this.viewTail=Promise.resolve();this.controller=null;this.drag=null;this.events();this.timer=setInterval(()=>this.tick(),100);}
 set(terrain,resolved){this.terrain=terrain;this.resolved=resolved;this.draw()}
 defaults(){const p=this.resolved?.experiment.spawn.position||[0,0,.65];return{enabled:true,follow:true,azimuth:-2.35,elevation:.4,distance:5,target:[p[0],p[1],.5]}}
 wanted(){return !!(this.autoConnect&&W.state.boot?.capabilities?.web_renderer&&W.state.online&&W.state.current?.owned&&W.state.current?.bridge_live&&W.state.page==='simulation'&&!document.hidden)}
 position(){return W.signalFresh('model')?W.state.current.telemetry.signals.model.data.position:null}
 enqueue(id,config,force=false){const gen=this.generation;this.viewTail=this.viewTail.catch(()=>{}).then(()=>{if(!force&&(gen!==this.generation||!this.wanted()))return;return W.api(`/api/runs/${encodeURIComponent(id)}/view`,config,5000)});return this.viewTail}
 clearFrame(){if(this.url){URL.revokeObjectURL(this.url);this.url=null}$('render-image').removeAttribute('src');$('render-image').hidden=true;this.sequence=-1;this.frameTimes=[]}
 disconnect(){const id=this.id,was=this.active;this.active=false;this.generation++;clearTimeout(this.frameTimer);this.frameTimer=null;this.controller?.abort();this.controller=null;this.clearFrame();this.connection='idle';this.lastSent=0;this.cameraNextAt=0;this.nextFetchAt=0;if(was&&id&&this.config)this.enqueue(id,{...this.config,enabled:false},true).catch(()=>{});}
 connect(){this.autoConnect=true;this.draw();this.tick();return{connected:this.connection==='streaming',kind:this.kind}}
 resetCamera(){this.config=this.defaults();this.error='';this.lastSent=0;this.draw();this.tick()}
 // Control acknowledgements must never block image reception or decoding.
 sendCamera(){
  const now=Date.now();if(this.sending||now<this.cameraNextAt||now-this.lastSent<=1800)return;
  const id=this.id,gen=this.generation;this.sending=true;this.lastSent=now;this.cameraNextAt=now+33;
  this.enqueue(id,{...this.config,enabled:true}).then(r=>{if(gen===this.generation&&r?.accepted!==true)this.error='相机请求尚未确认'}).catch(e=>{if(gen===this.generation)this.error=e.message}).finally(()=>{this.sending=false});
 }
 tick(){if(this.disposed)return;this.draw();if(!this.active)return;this.sendCamera();this.requestFrame();return this.viewTail}
 async requestFrame(){
  const render=W.state.current?.render;
  if(this.disposed||!this.active||this.fetching||Date.now()<this.nextFetchAt||!render||!render.frame_received_at||render.state==='failed'||Date.now()/1000-render.frame_received_at>3)return;
  clearTimeout(this.frameTimer);this.frameTimer=null;
  const id=this.id,gen=this.generation,wait=!!W.state.boot?.capabilities?.render_frame_wait;
  this.fetching=true;this.lastFetch=Date.now();const ac=new AbortController();this.controller=ac;
  const timeout=setTimeout(()=>ac.abort(),2500);let url,delay=0;
  try{
   const query=wait?`after=${this.sequence}&wait_ms=800`:`sequence=${render.sequence}`;
   const networkStart=performance.now();const r=await fetch(`/api/runs/${encodeURIComponent(id)}/view/frame?${query}`,{signal:ac.signal,cache:'no-store'});
   if(r.status===204)return;
   if(!r.ok){let detail;try{detail=await r.json()}catch{}throw new Error(detail?.error||'图像请求失败 '+r.status)}
   const sequence=Number(r.headers.get('X-Render-Sequence')),received=Number(r.headers.get('X-Render-Received'));
   const blob=await r.blob();this.lastNetworkMs=performance.now()-networkStart;
   if(Number.isFinite(sequence)&&sequence>0&&sequence<=this.sequence)return;
   if(blob.type!=='image/jpeg'||blob.size>2*1024*1024)throw new Error('相机图像格式无效');
   const decodeStart=performance.now();url=URL.createObjectURL(blob);const image=new Image();image.src=url;await image.decode();
   if(gen!==this.generation||!this.active){URL.revokeObjectURL(url);return}
   const previous=this.url;this.url=url;$('render-image').src=url;this.sequence=Number.isFinite(sequence)&&sequence>0?sequence:render.sequence;this.displayTime=received>0?received:render.frame_received_at;
   this.lastDecodeMs=performance.now()-decodeStart;this.displayedFrames++;this.frameTimes.push(performance.now());
   const now=performance.now();while(this.frameTimes.length>180||(this.frameTimes.length>1&&now-this.frameTimes[0]>5000))this.frameTimes.shift();
   this.error='';if(previous)URL.revokeObjectURL(previous);else this.draw();
  }catch(e){delay=200;if(gen===this.generation&&e.name!=='AbortError')this.error=e.message;if(url&&url!==this.url)URL.revokeObjectURL(url)}
  finally{
   clearTimeout(timeout);this.fetching=false;if(this.controller===ac)this.controller=null;
   if(gen===this.generation&&this.active&&!this.disposed){
    // One in-flight image, and no frame history: pull the newest frame on completion.
    // The legacy server fallback is capped too, without claiming duplicate frames.
    delay=Math.max(delay,wait?0:1000/30-(Date.now()-this.lastFetch));this.nextFetchAt=Date.now()+delay;
    this.frameTimer=setTimeout(()=>this.requestFrame(),delay);
   }
  }
 }
 frameStats(){
  const a=this.frameTimes,gaps=a.slice(1).map((v,i)=>v-a[i]).sort((x,y)=>x-y);
  return{fps:a.length>1?(a.length-1)*1000/(a.at(-1)-a[0]):null,p95GapMs:gaps.length?gaps[Math.ceil(gaps.length*.95)-1]:null,maxGapMs:gaps.at(-1)??null,decodeMs:this.lastDecodeMs??null,networkMs:this.lastNetworkMs??null,frameAgeMs:this.url?Math.max(0,Date.now()/1000-this.displayTime)*1000:null};
 }
 draw(){if(this.disposed)return;const id=W.state.selected||'';if(id!==this.id){this.disconnect();this.id=id;this.config=this.defaults();this.error=''}if(!this.config)this.config=this.defaults();const wanted=this.wanted();if(!wanted&&this.active)this.disconnect();if(wanted&&!this.active){this.active=true;this.connection='loading';this.lastSent=0}
  const s=W.state.current,render=s?.render,paused=s?.telemetry?.signals?.controller?.data.simulation_running===false,fresh=!!(this.url&&Date.now()/1000-this.displayTime<3),show=!!(this.active&&this.url&&(fresh||paused)&&render?.state!=='failed');$('render-image').hidden=!show;document.querySelector('.render-placeholder').hidden=show;
  let text='等待启动仿真',tone='',description='启动后自动接入 Gazebo 原生相机画面。';if(!W.state.boot?.capabilities?.web_renderer){text='渲染服务未接入';description='请更新并重启网页服务以启用渲染接口。'}else if(!W.state.online){text='画面连接中断';description='服务断线，停止显示旧画面；机器人状态以服务端为准。'}else if(s&&!s.owned){text=s.metadata.state==='prepared'?'场景已准备':'仿真未运行';description='启动当前任务后连接真实场景画面。'}else if(this.active){if(show){text=paused?'仿真暂停 · 最后一帧':'Gazebo 实时画面';tone=paused?'warn':'good';description=''}else if(this.error||render?.state==='failed'){text='渲染连接失败';tone='bad';description=this.error||render.error}else if(paused){text='仿真已暂停';description='等待相机图像；暂停时图像可能停止更新。'}else if(this.url||render?.frame_received_at){text='画面过期';tone='warn';description='未收到新图像，请检查 Gazebo 渲染和桥接日志。'}else{text='正在连接真实场景';description=render?.error||'正在创建观察相机，等待 Gazebo 渲染图像…'}}
  this.connection=show?(paused?'paused':'streaming'):(this.active?'loading':'idle');W.pill('scene-live',text,tone);$('view-state').textContent=text;$('view-description').textContent=description;$('camera-follow').classList.toggle('active',this.config.follow);$('camera-follow').setAttribute('aria-pressed',String(this.config.follow));for(const n of ['camera-follow','camera-overview','reset-view'])$(n).disabled=!this.wanted();
 }
 changed(){this.lastSent=0;this.draw();this.tick()}
 events(){const mount=$('renderer-mount');mount.oncontextmenu=e=>e.preventDefault();mount.onpointerdown=e=>{if(!this.active)return;e.preventDefault();mount.focus();const pan=e.button!==0||e.shiftKey;if(pan&&this.config.follow){const p=this.position();if(p)this.config.target=p.slice();this.config.follow=false}this.drag={pointer:e.pointerId,x:e.clientX,y:e.clientY,pan,base:W.clone(this.config)};mount.setPointerCapture(e.pointerId)};mount.onpointermove=e=>{const d=this.drag;if(!d||d.pointer!==e.pointerId)return;const dx=e.clientX-d.x,dy=e.clientY-d.y;if(d.pan){const a=d.base.azimuth,el=d.base.elevation,right=[-Math.sin(a),Math.cos(a),0],up=[-Math.sin(el)*Math.cos(a),-Math.sin(el)*Math.sin(a),Math.cos(el)];this.config.target=d.base.target.map((v,i)=>clamp(v+(-dx*right[i]+dy*up[i])*d.base.distance*.002,-10000,10000));}else{this.config.azimuth=azimuth(d.base.azimuth-dx*.006);this.config.elevation=clamp(d.base.elevation+dy*.006,.05,1.5)}this.changed()};for(const event of ['pointerup','pointercancel','lostpointercapture'])mount.addEventListener(event,()=>this.drag=null);mount.addEventListener('wheel',e=>{if(!this.active)return;e.preventDefault();this.config.distance=clamp(this.config.distance*Math.exp(e.deltaY*.001),.5,250);this.changed()},{passive:false});$('camera-follow').onclick=()=>{this.config.follow=!this.config.follow;if(!this.config.follow){const p=this.position();if(p)this.config.target=p.slice()}this.changed()};$('camera-overview').onclick=()=>{this.config={...this.defaults(),follow:false,target:[0,0,0],distance:40,elevation:.9};this.changed()};document.addEventListener('visibilitychange',()=>this.draw());window.addEventListener('pagehide',()=>this.disconnect());}
 dispose(){this.disconnect();this.disposed=true;clearInterval(this.timer)}
}
W.GazeboCameraRenderer=GazeboCameraRenderer;W.renderer=new GazeboCameraRenderer();
})();
