// Server-side MP4 capture; no browser-sized video buffers or control requests.
(()=>{'use strict';
const W=Workbench,{$,state}=W;
class ScreenRecording{
 constructor(){this.busy=false;$('video-toggle').onclick=()=>this.toggle().catch(e=>W.notice(e.message));this.draw()}
 async toggle(){
  if(this.busy)return;
  const id=state.selected,action=state.current?.video?.state==='recording'?'stop':'start';
  if(!state.online||!state.current?.owned)throw new Error('请先连接正在运行的仿真。');
  if(action==='start'&&(state.mode!=='live'||!state.boot?.capabilities?.video_recording))throw new Error('当前服务未启用仿真录屏，请重启网页服务并确认已安装 FFmpeg。');
  this.busy=action;this.draw();
  try{
   const result=await W.api(`/api/runs/${encodeURIComponent(id)}/video`,{action},35000);
   if(id===state.selected&&state.current){
    state.current.video=result;
    state.current.files=[...new Set([...(state.current.files||[]),...(result.files||[]).map(f=>f.name)])];
   }
   if(result.state==='failed')throw new Error(result.error||'视频编码失败');
   if(action==='stop'&&result.state!=='finished'&&result.state!=='idle')throw new Error('视频仍在写盘，请等待服务端确认。');
   W.notice(action==='start'?'仿真录屏已开始。':result.state==='idle'?'当前没有录屏。':result.directory?`MP4 已自动保存到 ${result.directory}/${result.name}`:'MP4 已完成写盘，可点击下载。',true);
   await W.service.poll();
  }finally{this.busy=false;W.renderStatus()}
 }
 draw(){
  const current=state.current,video=current?.video||{state:'idle',files:[]},recording=video.state==='recording';
  const unknown=!state.online&&['recording','finalizing','unknown'].includes(video.state);
  const finalizing=this.busy==='stop'||video.state==='finalizing';
  const render=current?.render,fresh=render?.frame_received_at&&Date.now()/1000-render.frame_received_at<3;
  const canStart=state.mode==='live'&&state.page==='simulation'&&state.boot?.capabilities?.video_recording&&current?.owned&&current?.bridge_live&&fresh&&render.state!=='failed';
  const button=$('video-toggle');button.disabled=!!this.busy||!state.online||finalizing||(!recording&&!canStart);
  button.classList.toggle('recording',recording);button.classList.toggle('loading',!!this.busy);
  button.title=finalizing?'正在完成 MP4 写盘':recording?'停止录屏并保存 MP4':!state.boot?.capabilities?.video_recording?'录屏未启用 · 请重启服务并确认 FFmpeg 可用':canStart?'开始录屏 · MP4':'等待实时仿真画面';
  button.setAttribute('aria-label',button.title);button.setAttribute('aria-pressed',String(recording));
  button.querySelector('use').setAttribute('href',recording?'#i-stop':'#i-record');
  const badge=$('video-state');badge.hidden=!unknown&&!finalizing&&!recording&&!['failed','finished','unknown'].includes(video.state);
  const duration=Math.max(0,(video.ended_at||Date.now()/1000)-(video.started_at||Date.now()/1000));
  const clock=Math.floor(duration/60)+':'+String(Math.floor(duration%60)).padStart(2,'0');
  W.pill('video-state',unknown||video.state==='unknown'?'录屏状态未知':finalizing?'MP4 写盘中':recording?'录屏 '+clock:video.state==='failed'?'录屏失败':'MP4 已保存',unknown?'warn':video.state==='failed'?'bad':recording?'bad':finalizing?'warn':'good');
  badge.title=video.error||video.name||'';
  const download=$('video-download'),file=video.files?.at(-1);download.hidden=!file;
  if(file){download.href=file.storage==='project'?`/api/videos/${encodeURIComponent(file.name)}`:`/api/runs/${encodeURIComponent(state.selected)}/files/${encodeURIComponent(file.name)}`;download.download=file.name;download.title='下载 MP4 · '+file.name}
 }
}
W.screenRecording=new ScreenRecording();
})();
