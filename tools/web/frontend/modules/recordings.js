// CSV sources and analysis view state; the live connection remains owned by service.js.
(() => {
 'use strict';
 const W=Workbench,{$}=W;
 class RecordingAnalysis{
  constructor(){
   this.store=null;this.source='';this.loading=false;this.controller=null;this.generation=0;this.files=[];this.views={};
   this.playing=false;this.playTimer=null;this.playhead=null;this.rate=1;
   $('load-recording').onclick=()=>this.open().catch(e=>this.error(e));
   $('refresh-recordings').onclick=()=>this.refresh().catch(e=>this.error(e));
   $('load-server-recording').onclick=()=>this.server().catch(e=>this.error(e));
   $('local-recording').onchange=()=>{const file=$('local-recording').files[0];if(file)this.local(file).catch(e=>this.error(e));$('local-recording').value=''};
   $('cancel-recording-load').onclick=()=>this.cancel();
   $('recording-dialog').addEventListener('close',()=>{if(this.loading&&this.loadInDialog&&!$('recording-dialog').open)this.cancel()});
   $('data-live').onclick=()=>this.live();
   $('replay').onclick=()=>{if(this.store)this.mode('replay');else this.open().catch(e=>this.error(e))};
   $('record-segment').onchange=()=>this.segment(Number($('record-segment').value));
   $('record-fit').onclick=()=>this.fit();
   $('record-position').oninput=()=>this.seek(Number($('record-position').value));
   $('record-play').onclick=()=>this.playing?this.pausePlayback():this.play();
   $('record-restart').onclick=()=>{this.seek(0);this.play()};
   $('record-speed').onchange=()=>{
    const rate=Number($('record-speed').value);if(![.25,.5,1,2,4].includes(rate))return;
    if(this.playing)this.tick();this.rate=rate;this.anchor();this.playbackControls();
   };
   document.addEventListener('visibilitychange',()=>{if(document.hidden)this.pausePlayback()});
   this.draw();
  }
  error(error){if(error.name==='AbortError')return;W.feedback('recording-load-error',error.message);W.notice(error.message)}
  async open(){W.feedback('recording-load-error','');if(!$('recording-dialog').open)$('recording-dialog').showModal();await this.refresh()}
  async refresh(){
   const chosen=$('server-recording').value;
   $('refresh-recordings').disabled=true;
   try{
    this.files=await W.api('/api/recordings',undefined,10000);
    W.options($('server-recording'),this.files.map((file,i)=>[String(i),`${file.run_id} / ${file.name} · ${(file.bytes/1024).toFixed(1)} KB`]),chosen);
    $('recording-catalog-hint').textContent=this.files.length?`${this.files.length} 个已完成 CSV；不包含正在写入的文件。`:'服务端暂无已完成 CSV，可选择本地 CSV 文件。';
    $('load-server-recording').disabled=this.loading||!this.files.length;
   }finally{$('refresh-recordings').disabled=false}
  }
  begin(){
   this.pausePlayback();this.cancel();const generation=++this.generation;
   this.controller=new AbortController();this.loading=true;this.loadInDialog=$('recording-dialog').open;
   W.feedback('recording-load-error','');$('recording-progress').textContent='正在加载 CSV…';this.draw();
   return {generation,signal:this.controller.signal};
  }
  cancel(){const loading=this.loading;this.generation++;this.controller?.abort();this.controller=null;this.loading=false;if(loading)$('recording-progress').textContent='已取消加载，原数据保留。';this.draw()}
  progress(loaded,total){$('recording-progress').textContent=`已读取 ${(loaded/1024).toFixed(1)} KB${total?' / '+(total/1024).toFixed(1)+' KB':''}，正在解析…`}
  async server(){
   const file=this.files[Number($('server-recording').value)];if(!file)throw new Error('请选择已完成的 CSV 录包。');
   const task=this.begin();
   try{
    const response=await fetch(`/api/runs/${encodeURIComponent(file.run_id)}/files/${encodeURIComponent(file.name)}`,{signal:task.signal});
    if(!response.ok){const error=await response.json().catch(()=>({}));throw new Error(error.error||'录包不可用，可能已被任务保留规则清理，请刷新列表。')}
    const store=await W.CsvImport.read(response.body,{signal:task.signal,bytes:Number(response.headers.get('Content-Length'))||file.bytes,onProgress:(...args)=>this.progress(...args)});
    this.commit(store,`${file.run_id} / ${file.name}`,task.generation);
   }finally{this.done(task.generation)}
  }
  async local(file){
   if(!file.name.toLowerCase().endsWith('.csv'))throw new Error('请选择 CSV 文件。');
   const task=this.begin();
   try{
    const store=await W.CsvImport.read(file.stream(),{signal:task.signal,bytes:file.size,onProgress:(...args)=>this.progress(...args)});
    this.commit(store,'本地 · '+file.name,task.generation);
   }finally{this.done(task.generation)}
  }
  done(generation){if(generation!==this.generation)return;this.loading=false;this.controller=null;this.draw()}
  commit(store,source,generation){
   if(generation!==this.generation)return;
   const previousMode=W.state.mode;
   if(previousMode==='replay')this.views.replay=null;
   this.pausePlayback();this.playhead=null;this.store=store;this.source=source;this.segmentKey=null;
   this.mode('replay',true);this.loading=false;this.controller=null;$('recording-dialog').close();
   $('recording-progress').textContent='';W.notice(`已加载 ${store.frames.length.toLocaleString()} 行、${store.segments.length} 段录包数据${store.skipped?'；跳过 '+store.skipped+' 行无有效时间数据':''}。`,true);
  }
  capture(){const c=W.charts;return c?{span:c.span,end:c.end,cursor:c.cursor,paused:c.paused,frozen:c.frozen,selected:new Set(c.selected),query:$('signal-search').value,playhead:this.playhead,epoch:this.store?.epoch}:null}
  mode(mode,fresh=false){
   if(mode==='replay'&&!this.store)return;
   const previous=W.state.mode;
   if(previous===mode&&!fresh){this.draw();return}
   this.pausePlayback();
   if(previous!==mode)this.views[previous]=this.capture();
   if(mode==='replay')W.joystick?.disable();
   W.state.mode=mode;
   const chart=W.charts,view=!fresh&&this.views[mode];
   if(mode==='replay'){
    if(view&&this.store.byEpoch.has(view.epoch))this.store.epoch=view.epoch;
    this.playhead=view?.playhead??null;
   }
   if(chart){
    chart.paused=view?.paused||false;chart.frozen=view?.frozen||null;chart.end=view?.end??null;chart.cursor=view?.cursor??null;
    chart.span=view?.span||Number(W.prefs.window)||30;
    chart.selected=view?.selected||new Set();
    $('signal-search').value=view?.query||'';
    chart.windowLabel();for(const card of chart.cards.values())card.legendKey=null;chart.signalsChanged();
    if(mode==='replay'&&fresh)this.segment(this.store.segments[0]);
    chart.syncDisplayControls();
   }
   W.renderStatus?.();this.draw();
  }
  live(){this.cancel();this.mode('live')}
  segment(epoch){
   if(!this.store?.byEpoch.has(epoch))return;
   this.pausePlayback();this.playhead=null;
   this.store.epoch=epoch;W.charts.paused=false;W.charts.frozen=null;this.fit();
   W.charts.syncDisplayControls();
   this.draw();
  }
  fit(){
   const frames=this.store?.currentEpoch();if(!frames?.length)return;
   this.pausePlayback();this.playhead=null;W.charts.interactions.clear();W.charts.paused=false;W.charts.frozen=null;
   W.charts.span=Math.max(1,frames.at(-1).t-frames[0].t);W.charts.end=frames.at(-1).t;W.charts.cursor=null;W.charts.windowLabel();W.charts.invalidate();
   $('record-position').max=frames.length-1;$('record-position').value=frames.length-1;this.position();this.playbackControls();
   W.charts.syncDisplayControls();
  }
  // Playback only advances the shared CSV view; it never publishes robot commands.
  indexAt(time){
   const frames=this.store?.currentEpoch()||[];let lo=0,hi=frames.length;
   while(lo<hi){const mid=(lo+hi)>>1;if(frames[mid].t<=time)lo=mid+1;else hi=mid}
   return Math.max(0,lo-1);
  }
  displayFrames(){
   const frames=this.store?.currentEpoch()||[];
   return this.playhead===null?frames:frames.slice(0,this.indexAt(this.playhead)+1);
  }
  anchor(){this.playStart=this.playhead;this.playClock=performance.now()}
  seek(index){
   if(W.state.mode!=='replay')return;
   const frames=this.store?.currentEpoch()||[],frame=frames[Math.max(0,Math.min(frames.length-1,Math.trunc(index)))];if(!frame)return;
   this.applyTime(frame.t);this.anchor();this.playbackControls();
  }
  applyTime(time){
   const frames=this.store?.currentEpoch()||[];if(!frames.length)return;
   this.playhead=Math.max(frames[0].t,Math.min(frames.at(-1).t,time));
   W.charts.paused=false;W.charts.frozen=null;W.charts.end=this.playhead;W.charts.cursor=this.playhead;W.charts.invalidate();
   $('record-position').value=this.indexAt(this.playhead);this.position();
   W.charts.syncDisplayControls();
  }
  play(){
   if(W.state.mode!=='replay'||W.state.page!=='data'||document.hidden||this.loading)return;
   const frames=this.store?.currentEpoch()||[];if(frames.length<2||frames.at(-1).t<=frames[0].t)return;
   if(this.playing)return;
   let time=this.playhead??frames[Number($('record-position').value)]?.t??frames[0].t;
   if(time>=frames.at(-1).t)time=frames[0].t;
   this.applyTime(time);this.anchor();this.playing=true;
   this.playTimer=setInterval(()=>this.tick(),50);this.playbackControls();
  }
  tick(now=performance.now()){
   if(!this.playing)return;
   if(W.state.mode!=='replay'||W.state.page!=='data'||document.hidden){this.pausePlayback();return}
   const frames=this.store.currentEpoch(),end=frames.at(-1).t;
   const time=this.playStart+Math.max(0,now-this.playClock)/1000*this.rate;
   this.applyTime(time);if(time>=end)this.pausePlayback();
  }
  pausePlayback(){
   clearInterval(this.playTimer);this.playTimer=null;this.playing=false;this.playbackControls();
  }
  playbackControls(){
   const frames=W.state.mode==='replay'?this.store?.currentEpoch()||[]:[],playable=frames.length>1&&frames.at(-1).t>frames[0].t;
   const status=this.playing?'播放中':!playable?'当前段无可播放时长':this.playhead!==null&&this.playhead>=frames.at(-1).t?'本段播放完成':this.playhead!==null?'播放已暂停':'等待播放';
   const key=[playable,this.loading,this.rate,status].join('|');if(key===this.controlsKey)return;this.controlsKey=key;
   $('record-play').disabled=!playable||this.loading;$('record-restart').disabled=!playable||this.loading;
   $('record-speed').disabled=!playable||this.loading;$('record-speed').value=String(this.rate);
   $('record-play').replaceChildren(W.icon(this.playing?'pause':'play'),document.createTextNode(this.playing?'暂停播放':'播放'));
   $('record-play').setAttribute('aria-pressed',String(this.playing));
   $('record-play-state').textContent=status;
  }
  position(){
   const frames=this.store?.currentEpoch()||[];if(!frames.length)return;
   if(this.playhead!==null)$('record-position').value=this.indexAt(this.playhead);
   const index=Math.max(0,Math.min(frames.length-1,Number($('record-position').value)||0));
   $('record-position-label').textContent=`${this.store.timeBasis==='simulation'?'仿真':'接收（相对）'} ${W.fmt(this.playhead??frames[index].t,3)} / ${W.fmt(frames.at(-1).t,3)} s`;
  }
  draw(){
   const replay=W.state.mode==='replay';
   $('data-live').classList.toggle('active',!replay);$('data-live').setAttribute('aria-pressed',String(!replay));
   $('replay').classList.toggle('active',replay);$('replay').setAttribute('aria-pressed',String(replay));
   $('recording-analysis').hidden=!replay;$('recording-source-state').textContent=replay?'CSV 录包分析':'实时遥测';
   $('recording-source-state').className='pill '+(replay?'warn':'');
   $('recording-source-name').textContent=replay?this.source:'当前运行会话';$('recording-source-name').title=replay?this.source:'';
   $('load-server-recording').disabled=this.loading||!this.files.length;
   $('local-recording').disabled=this.loading;$('cancel-recording-load').hidden=!this.loading;
   if(replay&&this.store){
    const current=String(this.store.epoch),key=this.source+'|'+this.store.frames.length;
    if(this.segmentKey!==key){this.segmentKey=key;W.options($('record-segment'),this.store.segments.map((epoch,i)=>{const a=this.store.byEpoch.get(epoch);return[String(epoch),`第 ${i+1} 段 · ${W.fmt(a[0].t,2)}–${W.fmt(a.at(-1).t,2)} s · ${a.length} 行`]}),current);}
    $('record-segment').value=current;$('record-position').max=this.store.currentEpoch().length-1;
    $('buffer-info').textContent=`录包 ${this.store.frames.length.toLocaleString()} 行 / ${this.store.descriptors.length} 信号`;
    this.position();this.playbackControls();
   }else if(W.telemetry)$('buffer-info').textContent=`页面缓冲 ${W.telemetry.frames.length} / ${W.telemetry.max}`;
  }
 }
 W.recordings=new RecordingAnalysis();
})();
