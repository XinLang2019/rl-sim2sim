// Canvas workspaces over one shared sample store, with explicit gaps and units.
(()=>{"use strict";const W=window.Workbench,{$,el,fmt}=W;
const palette=['#3478db','#bc791e','#9760c9','#188b76','#cf507c','#3c87ad','#9a7b31','#7d83cc'];function color(id){let h=0;for(const c of id)h=(h*31+c.charCodeAt(0))>>>0;return palette[h%palette.length]}
function decimate(frames,id,pixels){if(frames.length<=pixels*2)return frames;const out=[],step=Math.ceil(frames.length/Math.max(1,pixels));for(let i=0;i<frames.length;i+=step){const bucket=frames.slice(i,i+step);let lo=null,hi=null;for(const f of bucket){const v=f.values[id];if(!W.finite(v)){out.push(f);continue}if(!lo||v<lo.values[id])lo=f;if(!hi||v>hi.values[id])hi=f}if(lo)out.push(lo);if(hi&&hi!==lo)out.push(hi)}return [...new Set(out)].sort((a,b)=>a.wall-b.wall)}
class ChartWorkspace{constructor(){this.cards=new Map();this.selected=new Set();this.paused=false;this.frozen=null;this.end=null;this.span=Number(W.prefs.window)||30;this.cursor=null;this.dirty=true;this.disposer=new AbortController();this.lastFrame=0;this.loop=setInterval(()=>{if(this.dirty&&W.state.page==='data'){this.dirty=false;this.draw()}},125);this.interactions=new W.ChartInteraction(this);this.layout=new W.ChartLayout(this);this.events();this.rebuild();}
 invalidate(){this.dirty=true}frames(){return this.paused?this.frozen||[]:W.state.mode==='replay'?W.recordings.displayFrames():W.chartData().currentEpoch()}bounds(){const a=this.frames(),latest=a.at(-1)?.t??0;let end=this.end??latest,start=end-this.span;if(W.state.mode==='replay'&&W.recordings.playhead!==null&&this.end===W.recordings.playhead&&a.length){start=Math.max(a[0].t,start);end=start+this.span}return {start,end}}follow(){this.interactions.clear();if(!this.paused)this.end=null;this.cursor=null;this.syncDisplayControls();this.invalidate()}signalsChanged(){this.signals();this.invalidate()}
 syncDisplayControls(){
  const button=$('pause-display'),label=this.paused?'继续显示':'暂停显示';
  if(button.textContent!==label)button.replaceChildren(W.icon(this.paused?'play':'pause'),document.createTextNode(label));
  button.setAttribute('aria-pressed',String(this.paused));
 }
 pauseDisplay(){
  if(W.state.mode==='replay')W.recordings.pausePlayback();
  if(!this.paused){this.frozen=this.frames().slice();this.paused=true}
  this.syncDisplayControls();this.invalidate();
 }
 resumeDisplay({playReplay=false}={}){
  this.paused=false;this.frozen=null;this.end=null;this.cursor=null;this.interactions.clear();
  this.syncDisplayControls();this.invalidate();
  if(playReplay&&W.state.mode==='replay')W.recordings.play();
 }
 rebuild(){this.layout.rebuild()}
 createCard(config){
  config.hidden=config.hidden||[];
  const card=el('section','chart-card'),head=el('div','chart-head'),tools=el('div','chart-tools');
  card.dataset.chartId=config.id;card.tabIndex=0;card.setAttribute('aria-label',config.title);
  const tool=(label,name,callback)=>{const b=el('button');b.title=label;b.setAttribute('aria-label',label);b.append(W.icon(name));b.onclick=callback;tools.append(b);return b};
  tool('编辑画布','sliders',()=>W.chartEditor.open(config));
  tool('切换曲线 / 数值卡片','chart',()=>{config.kind=config.kind==='value'?'line':'value';this.persist();this.rebuild()});
  tool('左右分割','split-row',()=>this.layout.split(config.id,'row'));
  tool('上下分割','split-column',()=>this.layout.split(config.id,'column'));
  const fitButton=tool('显示全部数据（当前数据段）','fit-data',()=>this.interactions.fit(config.id));
  fitButton.hidden=config.kind==='value';fitButton.disabled=!config.signals.length||!this.interactions.fitFrames().length;
  tool('最大化画布','expand',()=>{card.classList.toggle('maximized');this.invalidate()});
  tool('删除画布','x',()=>this.remove(config.id));
  const legend=el('div','chart-legend'),plot=el('div','plot-wrap'),canvas=el('canvas'),message=el('div','chart-message','拖入左侧信号'),tooltip=el('div','chart-tooltip'),selection=el('div','chart-selection');tooltip.hidden=true;selection.hidden=true;selection.setAttribute('aria-hidden','true');plot.append(canvas,message,tooltip,selection);
  const status=el('span','chart-status'),unitLabel=el('span','chart-unit');status.hidden=true;head.append(legend,status,unitLabel,tools);
  const values=el('div','chart-values');values.hidden=true;card.append(head,plot,values);
  const observer=new ResizeObserver(()=>this.invalidate());observer.observe(card);this.cards.set(config.id,{config,card,canvas,message,tooltip,legend,values,plot,status,unitLabel,observer,selection,fitButton});this.interactions.attach(this.cards.get(config.id));
  card.addEventListener('focusin',()=>this.layout.select(config.id));card.addEventListener('pointerdown',()=>this.layout.select(config.id));
  const accepts=e=>[...e.dataTransfer.types].includes('application/x-rlsim-signals');
  card.ondragover=e=>{if(accepts(e)){e.preventDefault();e.dataTransfer.dropEffect='copy';card.classList.add('signal-drop-target')}};
  card.ondragleave=e=>{if(!card.contains(e.relatedTarget))card.classList.remove('signal-drop-target')};
  card.ondrop=e=>{card.classList.remove('signal-drop-target');if(!accepts(e))return;e.preventDefault();try{const ids=JSON.parse(e.dataTransfer.getData('application/x-rlsim-signals'));if(!Array.isArray(ids)||ids.some(id=>typeof id!=='string'))return;if(this.assignSignals(ids,config.id))this.clearSelection()}catch(error){W.notice('无法添加所拖拽的信号：'+error.message)}};

  return card;
 }
 windowLabel(){const select=$('time-window');for(const o of [...select.options])if(o.dataset.custom)o.remove();if(![...select.options].some(o=>Number(o.value)===this.span)){const o=new Option(fmt(this.span,Math.max(1,Math.min(6,Math.ceil(-Math.log10(this.span))+2)))+' 秒',String(this.span));o.dataset.custom='true';select.add(o)}select.value=String(this.span)}
 persist(){W.save()}
 targets(){const chosen=$('chart-target').value;W.options($('chart-target'),[['new','新 Tab（按单位分组）'],...W.prefs.charts.map(c=>[c.id,(this.layout.tabFor(c.id)?.title||'Tab')+' / '+c.title])],chosen)}
 remove(id){this.layout.remove(id)}
 add(title,signals=[],kind='line'){
  const config={id:crypto.randomUUID(),title,signals,kind};W.prefs.charts.push(config);this.layout.add(config);this.persist();this.rebuild();$('chart-target').value=config.id;return config.id;
 }
 signals(){
  const query=$('signal-search').value.trim(),list=$('signal-list');
  if(!this.signalOpen)this.signalOpen=new Map();
  if(!this.signalSearching)for(const n of list.querySelectorAll('details[data-signal-path]'))this.signalOpen.set(n.dataset.signalPath,n.open);
  this.signalSearching=!!query;
  const root={children:new Map(),items:[],count:0};
  for(const d of W.chartData().descriptors){
   if(!W.signalMatches(d,query))continue;
   let node=root;node.count++;
   for(const part of [d.group,...(d.path||[])]){
    if(!node.children.has(part))node.children.set(part,{name:part,children:new Map(),items:[],count:0});
    node=node.children.get(part);node.count++;
   }
   node.items.push(d);
  }
  const branch=(node,path)=>{
   const group=el('details',path.length===1?'signal-group':'signal-subgroup'),key=JSON.stringify(path);
   group.dataset.signalPath=key;group.open=!!query||(this.signalOpen.get(key)??(path.length===1&&['机体状态','关节状态'].includes(node.name)));
   const summary=el('summary','signal-group-title');summary.append(el('span','signal-tree-label',node.name),el('small','signal-tree-count',node.count));group.append(summary);
   group.addEventListener('toggle',()=>{if(!query&&group.isConnected)this.signalOpen.set(key,group.open)});
   const children=el('div','signal-children');let nodes=[...node.children.values()];
   if(path.length===1&&node.name==='关节状态'){
    const order=['FL · 左前腿','FR · 右前腿','RL · 左后腿','RR · 右后腿','其他关节'];nodes.sort((a,b)=>order.indexOf(a.name)-order.indexOf(b.name));
   }
   for(const child of nodes)children.append(branch(child,[...path,child.name]));
   for(const d of node.items){
    const label=el('label','signal-row'),check=el('input');check.type='checkbox';check.checked=this.selected.has(d.id);check.value=d.id;
    check.onchange=()=>{check.checked?this.selected.add(d.id):this.selected.delete(d.id);this.selectionCount()};
    label.draggable=true;label.dataset.signalId=d.id;label.ondragstart=e=>{const ids=this.selected.has(d.id)?[...this.selected]:[d.id];e.dataTransfer.setData('application/x-rlsim-signals',JSON.stringify(ids));e.dataTransfer.setData('text/plain',ids.join(', '));e.dataTransfer.effectAllowed='copy'};
    label.title=[d.label,d.frame,d.id].join(' · ')+' · 拖入画布';check.setAttribute('aria-label',d.label);
    const name=el('span','signal-drag-name',d.shortLabel||d.label);name.draggable=true;label.append(check,name,el('small','',d.unit||'无量纲'));children.append(label);
   }
   group.append(children);return group;
  };
  list.replaceChildren(...[...root.children.values()].map(node=>branch(node,[node.name])));
  if(W.signalMatches({id:'contact',group:'接触信息',label:'当前后端未发布接触信号。'},query)){
   const contact=el('details','signal-group'),summary=el('summary','signal-group-title');summary.append(el('span','signal-tree-label','接触信息'),el('small','signal-tree-count','未接入'));
   contact.append(summary,el('p','hint',W.state.mode==='replay'?'该录包未包含接触信号。':'当前后端未发布接触信号。'));list.append(contact);
  }
  if(!list.children.length)list.append(el('p','hint signal-empty','未找到匹配信号。'));
  this.selectionCount();
 }

 selectionCount(){$('signal-count').textContent=this.selected.size+' 个已选择'}
 clearSelection(){this.selected.clear();for(const check of $('signal-list').querySelectorAll('input[type=checkbox]'))check.checked=false;this.selectionCount()}
 addSelected(){
  if(!this.selected.size)return W.notice('先选择需要监测的信号。');
  if(this.assignSignals([...this.selected],$('chart-target').value))this.clearSelection();
 }
 assignSignals(ids,targetId){
  const groups=new Map();
  for(const id of new Set(ids)){const d=W.chartData().descriptor(id);if(!d)continue;if(!groups.has(d.unit))groups.set(d.unit,[]);groups.get(d.unit).push(id)}
  if(!groups.size){W.notice('所选信号不在当前数据源中，请重新选择。');return false}
  const target=W.prefs.charts.find(c=>c.id===targetId),units=target?.signals.map(id=>W.chartData().descriptor(id)?.unit).filter(v=>v!==undefined)||[];
  let assigned=false;
  const plans=[...groups].map(([unit,signals])=>{const match=target&&!assigned&&(!units.length||units.every(u=>u===unit));if(match)assigned=true;return{unit,target:match?target:null,signals:match?[...new Set([...target.signals,...signals])]:signals}});
  if(plans.some(p=>p.signals.length>32)){W.notice('每张画布最多 32 条信号，请分批加入。');return false}
  let anchor=target?.id;
  for(const plan of plans){
   if(plan.target){plan.target.signals=plan.signals;continue}
   const config={id:crypto.randomUUID(),title:plan.unit?`${plan.unit} 信号监测`:'action 信号',signals:plan.signals,kind:'line'};
   W.prefs.charts.push(config);
   if(anchor){this.layout.split(anchor,'column',config,false);anchor=config.id}else this.layout.add(config);
  }
  this.persist();this.rebuild();
  if(target){const tab=this.layout.tabFor(target.id);if(tab)this.layout.activate(tab.id)}
  if(groups.size>1)W.notice('不同单位已分到独立画布。',true);
  return true;
 }

 draw(){this.interactions.sync();const frames=this.frames(),fullFrames=W.state.mode==='replay'?W.recordings.store.currentEpoch():frames,{start,end}=this.interactions.displayBounds()||this.bounds();const visible=frames.filter(f=>f.t>=start&&f.t<=end);const sample=this.cursor===null?visible.at(-1):visible.reduce((a,b)=>!a||Math.abs(b.t-this.cursor)<Math.abs(a.t-this.cursor)?b:a,null);for(const c of this.cards.values()){const {config,canvas,legend,plot,values,tooltip}=c;if(c.card.closest('[hidden]'))continue;const ids=config.signals.filter(id=>W.chartData().descriptor(id));const unit=W.chartData().descriptor(ids[0])?.unit||'';const angleUnit=config.angleUnit||W.prefs.angle,show=v=>(unit==='rad'||unit==='rad/s')&&angleUnit==='deg'?v*180/Math.PI:v,displayUnit=unit==='rad'&&angleUnit==='deg'?'deg':unit==='rad/s'&&angleUnit==='deg'?'deg/s':unit;if(c.legendKey!==ids.join('|')){c.legendKey=ids.join('|');legend.replaceChildren(...ids.map(id=>{const d=W.chartData().descriptor(id),b=el('button','legend-item'+(config.hidden.includes(id)?' off':'')),sw=el('span','swatch');sw.style.background=color(id);b.title=d.label+' / '+d.frame;b.append(sw,el('span','legend-label',d.label.replace('joint.','')),el('span','legend-value',fmt(W.finite(sample?.values[id])?show(sample.values[id]):null)));b.onclick=()=>{config.hidden=config.hidden.includes(id)?config.hidden.filter(v=>v!==id):[...config.hidden,id];this.persist();this.invalidate()};return b}));}for(const [i,b]of [...legend.children].entries()){const id=ids[i];b.classList.toggle('off',config.hidden.includes(id));b.querySelector('.legend-value').textContent=fmt(W.finite(sample?.values[id])?show(sample.values[id]):null)}const replay=W.state.mode==='replay',stale=!replay&&!this.paused&&frames.length>0&&(!W.state.online||!W.signalFresh('clock'));c.card.classList.toggle('stale',stale);c.status.hidden=!stale&&!this.paused;c.status.textContent=this.paused?'显示暂停':'数据过期';
const basis=replay?'录包分析 · '+(W.recordings.store.timeBasis==='simulation'?'仿真时间':'接收时间（相对）'):'实时 · 仿真时间';
const description=config.title+' · '+basis+' / s'+(sample?' · 接收 '+(W.finite(sample.wall)?new Date(sample.wall*1000).toLocaleTimeString():'未提供')+' · 消息时间 '+(sample.messageTime??'未提供'):'');
c.card.setAttribute('aria-description',description);c.unitLabel.textContent=displayUnit;c.unitLabel.title='Y 轴：'+(displayUnit||'无量纲')+'；X 轴：'+description;
const numeric=config.kind==='value';c.fitButton.hidden=numeric;c.fitButton.disabled=!ids.length||!fullFrames.length;plot.hidden=numeric;values.hidden=!numeric;if(numeric){values.replaceChildren(...ids.map(id=>{const n=el('div');n.append(el('strong','',fmt(W.finite(sample?.values[id])?show(sample.values[id]):null)),el('small','',W.chartData().descriptor(id).label+' · '+displayUnit));return n}));continue}const w=canvas.clientWidth,h=canvas.clientHeight;if(w<80||h<60)continue;const ratio=devicePixelRatio||1;canvas.width=w*ratio;canvas.height=h*ratio;const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);const colors=W.colors(),text=colors('--chart-text'),grid=colors('--chart-grid');const l=52,r=w-10,top=8,bottom=h-23;ctx.font='10px -apple-system, sans-serif';ctx.fillStyle=text;ctx.textAlign='left';const active=ids.filter(id=>!config.hidden.includes(id));let min=Infinity,max=-Infinity,valueCount=0;for(const f of visible)for(const id of active){const v=f.values[id];if(W.finite(v)){const shown=show(v);min=Math.min(min,shown);max=Math.max(max,shown);valueCount++}}if(!valueCount){min=0;max=1}if(max===min){min-=.05;max+=.05}else{const pad=(max-min)*.12;min-=pad;max+=pad}// Manual ranges remain in the protocol's canonical units.
const scale=show(1),selectedRange=this.interactions.range(config.id,unit);
if(selectedRange){min=show(selectedRange.min);max=show(selectedRange.max)}
c.view={left:l,right:r,top,bottom,start,end,unit,range:{min:min/scale,max:max/scale}};
// Framed X/Y axes with major and minor ticks; grid is optional.
ctx.lineWidth=1;ctx.strokeStyle=text;ctx.beginPath();ctx.rect(l,top,r-l,bottom-top);ctx.stroke();
for(let i=0;i<=20;i++){
 const major=i%5===0,y=top+(bottom-top)*i/20;
 if(major&&W.prefs.chartGrid){ctx.strokeStyle=grid;ctx.beginPath();ctx.moveTo(l,y);ctx.lineTo(r,y);ctx.stroke()}
 ctx.strokeStyle=text;ctx.beginPath();ctx.moveTo(l,y);ctx.lineTo(l-(major?5:3),y);ctx.stroke();
 if(major){ctx.fillStyle=text;ctx.textAlign='right';ctx.fillText(fmt(max-(max-min)*i/20,Math.max(2,Math.min(5,Math.ceil(-Math.log10((max-min)/4))+1))),l-8,y+3)}
}
for(let i=0;i<=20;i++){
 const major=i%5===0,x=l+(r-l)*i/20;
 if(major&&W.prefs.chartGrid){ctx.strokeStyle=grid;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke()}
 ctx.strokeStyle=text;ctx.beginPath();ctx.moveTo(x,bottom);ctx.lineTo(x,bottom+(major?5:3));ctx.stroke();
 if(major){ctx.fillStyle=text;ctx.textAlign='center';ctx.fillText(fmt(start+(end-start)*i/20,Math.max(1,Math.min(6,Math.ceil(-Math.log10((end-start)/4))+1))),x,h-8)}
}
ctx.save();ctx.beginPath();ctx.rect(l,top,r-l,bottom-top);ctx.clip();
for(const id of active){ctx.strokeStyle=color(id);ctx.lineWidth=1.5;ctx.beginPath();let prev=null,only=null,points=0;for(const f of decimate(visible,id,Math.max(30,r-l))){const v=f.values[id];if(!W.finite(v)){prev=null;continue}const x=l+(f.t-start)/this.span*(r-l),y=bottom-(show(v)-min)/(max-min)*(bottom-top);only={x,y};points++;if(!prev||f.epoch!==prev.epoch||f.wall-prev.wall>1.25)ctx.moveTo(x,y);else ctx.lineTo(x,y);prev=f}ctx.stroke();if(points===1){ctx.fillStyle=color(id);ctx.beginPath();ctx.arc(only.x,only.y,2.5,0,Math.PI*2);ctx.fill()}}ctx.restore();c.message.hidden=valueCount>0;c.message.textContent=!ids.length?'将左侧信号拖入此画布':replay?'当前录包或所选时间段缺少此信号':!W.state.selected?'选择运行会话后显示真实数据':!W.state.current?.owned?'历史会话未接入回放':'等待真实遥测 / 数据缺失';if(!this.interactions.drag&&this.cursor!==null&&sample&&Math.abs(sample.t-this.cursor)<1.2){const x=l+(sample.t-start)/this.span*(r-l);ctx.strokeStyle=colors('--chart-cursor');ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke();ctx.setLineDash([]);tooltip.hidden=false;tooltip.replaceChildren(el('div','',`${replay&&W.recordings.store.timeBasis==='received'?'接收（相对）':'仿真'} ${fmt(sample.t,4)} s`),...active.map(id=>el('div','',`${W.chartData().descriptor(id).label}: ${fmt(W.finite(sample.values[id])?show(sample.values[id]):null,5)} ${displayUnit}`)));}else tooltip.hidden=true;}}
 export(){const ids=[...new Set(W.prefs.charts.flatMap(c=>c.signals))].slice(0,64);const quote=s=>'"'+String(s??'').replaceAll('"','""')+'"';const rows=[['segment','simulation_time_s','received_time_unix_s',...ids.map(id=>{const d=W.chartData().descriptor(id);return id+' ['+(d?.unit||'')+']'})]],f=W.chartData().frames;for(const x of f)rows.push([x.epoch,'simulationTime' in x?x.simulationTime:x.t,x.wall,...ids.map(id=>x.values[id]??'')]);const blob=new Blob(['\ufeff'+rows.map(r=>r.map(quote).join(',')).join('\n')],{type:'text/csv;charset=utf-8'});const url=URL.createObjectURL(blob),a=el('a');a.href=url;a.download=W.state.mode==='replay'?'rlsim-recording-analysis.csv':'rlsim-page-samples.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)}
 events(){$('signal-search').oninput=()=>this.signals();$('add-chart').onclick=()=>this.add('Tab '+(W.prefs.tabs.length+1));$('chart-grid-toggle').onchange=()=>{W.prefs.chartGrid=$('chart-grid-toggle').checked;this.persist();this.invalidate()};$('sample-settings').onclick=()=>{$('sample-hz').value=String(W.prefs.sampleHz);$('sample-dialog').showModal()};$('sample-form').onsubmit=e=>{e.preventDefault();const hz=Number($('sample-hz').value);if(![1,2,4,10].includes(hz))return;W.prefs.sampleHz=hz;W.service.setSampleHz(hz);$('sample-settings').lastChild.textContent='采样 '+hz+' Hz';this.persist();$('sample-dialog').close()};$('sample-settings').lastChild.textContent='采样 '+W.prefs.sampleHz+' Hz';$('add-signals').onclick=()=>this.addSelected();this.windowLabel();$('time-window').onchange=()=>{this.interactions.clear();this.span=Number($('time-window').value);W.prefs.window=this.span;this.end=null;this.persist();this.invalidate()};this.syncDisplayControls();$('pause-display').onclick=()=>{this.interactions.cancel();if(this.paused)this.resumeDisplay({playReplay:true});else this.pauseDisplay()};$('auto-follow').onclick=()=>this.resumeDisplay();$('export-csv').onclick=()=>this.export();$('apply-chart-preset').onclick=()=>{const kind=$('chart-preset').value;if(kind==='contact')return W.notice('当前后端没有接触遥测，不生成替代数据。');const joint=W.chartData().descriptors.find(d=>d.id.startsWith('joint.')&&d.id.endsWith('.q'))?.id;const sets=kind==='joints'&&joint?[{title:'关节位置 · 目标 / 实际',signals:[joint,joint.replace(/\.q$/,'.target_q')]},{title:'关节力矩',signals:[joint.replace(/\.q$/,'.effort')]}]:kind==='performance'?[{title:'推理与解码',signals:['performance.inference']},{title:'策略频率',signals:['performance.policy_hz']},{title:'RTF',signals:['performance.rtf']}]:[{title:'机身高度',signals:['body.z']},{title:'机体姿态',signals:['body.roll','body.pitch']}];for(const c of sets)this.add(c.title,c.signals);};}
 dispose(){this.interactions.clear();clearInterval(this.loop);for(const c of this.cards.values())c.observer.disconnect();this.disposer.abort();}
}
W.charts=new ChartWorkspace();W.SignalCharts=ChartWorkspace;W.chartColor=color;W.decimate=decimate;
})();
