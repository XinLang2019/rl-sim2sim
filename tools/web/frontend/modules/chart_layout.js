// Persistent tabs and binary split trees. Layout never owns a connection or data.
(()=>{'use strict';
 const W=Workbench,{$,el}=W;
 class ChartLayout{
  constructor(workspace){this.workspace=workspace;this.activeChart=null;this.dragTab=null}
  leaves(node){return !node?[]:node.chart?[node.chart]:node.children.flatMap(n=>this.leaves(n))}
  normalize(){
   const configs=new Map(W.prefs.charts.filter(c=>c&&typeof c.id==='string'&&Array.isArray(c.signals)).map(c=>[c.id,c]));
   W.prefs.charts=[...configs.values()];const used=new Set(),tabIds=new Set();
   const clean=node=>{
    if(!node||typeof node!=='object')return null;
    if(node.chart){if(!configs.has(node.chart)||used.has(node.chart))return null;used.add(node.chart);return{chart:node.chart}}
    if(!['row','column'].includes(node.axis)||!Array.isArray(node.children))return null;
    const children=node.children.slice(0,2).map(clean).filter(Boolean);
    return children.length===2?{axis:node.axis,ratio:W.finite(node.ratio)&&node.ratio>0&&node.ratio<1?node.ratio:.5,children}:children[0]||null;
   };
   W.prefs.tabs=(W.prefs.tabs||[]).filter(t=>t&&typeof t.id==='string'&&!tabIds.has(t.id)&&tabIds.add(t.id)).map(t=>({...t,title:String(t.title||'监测 Tab').slice(0,60),layout:clean(t.layout)})).filter(t=>t.layout);
   for(const tab of W.prefs.tabs)if(tab.layoutVersion!==2){
    const upgrade=node=>{
     if(node.chart)return;
     const run=[],boundaries=[];
     const collect=child=>{if(!child.chart&&child.axis===node.axis){run.push(child);child.children.forEach(collect)}else boundaries.push(child)};
     collect(node);
     // Migrate the old default 1/2 + 1/4 + 1/4 layout, retaining custom ratios.
     if(run.length>1&&run.every(n=>n.ratio===.5))this.equalize(node,node.axis);
     boundaries.forEach(upgrade);
    };
    upgrade(tab.layout);tab.layoutVersion=2;
   }
   // Migrate the previous flat card layout without losing signal selections.
   for(const c of configs.values())if(!used.has(c.id))W.prefs.tabs.push({id:crypto.randomUUID(),title:c.title||'监测 Tab',layoutVersion:2,layout:{chart:c.id}});
   if(!W.prefs.tabs.some(t=>t.id===W.prefs.activeTab))W.prefs.activeTab=W.prefs.tabs[0]?.id||null;
   const active=W.prefs.tabs.find(t=>t.id===W.prefs.activeTab);
   if(!this.leaves(active?.layout).includes(this.activeChart))this.activeChart=this.leaves(active?.layout)[0]||null;
  }
  tabFor(id){return W.prefs.tabs.find(t=>this.leaves(t.layout).includes(id))}
  activate(id){
   if(!W.prefs.tabs.some(t=>t.id===id))return;
   this.workspace.interactions.cancel();W.prefs.activeTab=id;
   if(!this.leaves(this.tabFor(this.activeChart)?.layout).length||this.tabFor(this.activeChart)?.id!==id)this.activeChart=this.leaves(W.prefs.tabs.find(t=>t.id===id).layout)[0];
   for(const n of $('chart-tabs').children){const on=n.dataset.tabId===id;n.classList.toggle('active',on);const b=n.querySelector('[role=tab]');b.setAttribute('aria-selected',String(on));b.tabIndex=on?0:-1}
   for(const n of $('chart-grid').children)n.hidden=n.dataset.tabId!==id;
   this.select(this.activeChart);W.save();this.workspace.invalidate();
  }
  select(id){this.activeChart=id;for(const [key,c]of this.workspace.cards)c.card.classList.toggle('selected-pane',key===id);if([...$('chart-target').options].some(o=>o.value===id))$('chart-target').value=id}
  add(config){const tab={id:crypto.randomUUID(),title:config.title,layoutVersion:2,layout:{chart:config.id}};W.prefs.tabs.push(tab);W.prefs.activeTab=tab.id;this.activeChart=config.id}
  axisCount(node,axis){return !node.chart&&node.axis===axis?node.children.reduce((n,c)=>n+this.axisCount(c,axis),0):1}
  equalize(node,axis){
   if(node.chart||node.axis!==axis)return;
   const counts=node.children.map(c=>this.axisCount(c,axis));node.ratio=counts[0]/(counts[0]+counts[1]);
   node.children.forEach(c=>this.equalize(c,axis));
  }
  split(id,axis='row',config=null,rebuild=true){
   const tab=this.tabFor(id);if(!tab||!['row','column'].includes(axis))return null;
   if(!config){config={id:crypto.randomUUID(),title:'监测画布',signals:[],kind:'line'};W.prefs.charts.push(config)}
   // Balance only the contiguous run in the requested direction. A perpendicular
   // split is one slot, so editing one column never changes neighboring columns.
   const pathTo=node=>{if(node.chart)return node.chart===id?[node]:null;for(const child of node.children){const path=pathTo(child);if(path)return[node,...path]}return null};
   const path=pathTo(tab.layout);let index=path.length-1;
   while(index>0&&path[index-1].axis===axis)index--;
   const group=path[index];
   const replace=node=>node.chart===id?{axis,ratio:.5,children:[node,{chart:config.id}]}:node.chart?node:{...node,children:node.children.map(replace)};
   const balanced=replace(group);this.equalize(balanced,axis);
   const install=node=>node===group?balanced:node.chart?node:{...node,children:node.children.map(install)};
   tab.layout=install(tab.layout);this.activeChart=config.id;W.prefs.activeTab=tab.id;
   if(rebuild){W.save();this.rebuild()}return config.id;
  }
  remove(id){
   const tab=this.tabFor(id);
   if(tab){
    const pathTo=node=>{if(node.chart)return node.chart===id?[node]:null;for(const child of node.children){const path=pathTo(child);if(path)return[node,...path]}return null};
    const path=pathTo(tab.layout);
    if(path.length===1)tab.layout=null;
    else{
     const axis=path[path.length-2].axis;let index=path.length-2;
     while(index>0&&path[index-1].axis===axis)index--;
     const group=path[index];
     // Promote surviving siblings before calculating equal shares. Keeping the
     // old ratio here is what left three panes with the former four-pane weights.
     const prune=node=>{
      if(node.chart)return node.chart===id?null:node;
      const children=node.children.map(prune).filter(Boolean);
      return children.length===2?{...node,children}:children[0]||null;
     };
     const balanced=prune(group);this.equalize(balanced,axis);
     const install=node=>node===group?balanced:node.chart?node:{...node,children:node.children.map(install)};
     tab.layout=install(tab.layout);
    }
   }
   W.prefs.charts=W.prefs.charts.filter(c=>c.id!==id);this.rebuild();W.save();
  }
  closeTab(id){const tab=W.prefs.tabs.find(t=>t.id===id);if(!tab)return;const ids=new Set(this.leaves(tab.layout));W.prefs.charts=W.prefs.charts.filter(c=>!ids.has(c.id));W.prefs.tabs=W.prefs.tabs.filter(t=>t!==tab);this.rebuild();W.save()}
  rename(tab,button){
   const input=el('input','tab-name-input');input.value=tab.title;input.maxLength=60;button.replaceWith(input);input.focus();input.select();
   const finish=save=>{if(!input.isConnected)return;if(save&&input.value.trim())tab.title=input.value.trim();W.save();this.rebuild()};
   input.onblur=()=>finish(true);input.onkeydown=e=>{if(e.key==='Enter'){e.preventDefault();finish(true)}if(e.key==='Escape')finish(false)};
  }
  separator(node,container,first,second){
   const handle=el('div','chart-splitter');handle.role='separator';handle.tabIndex=0;
   handle.setAttribute('aria-label',node.axis==='row'?'调整左右分割比例':'调整上下分割比例');handle.setAttribute('aria-orientation',node.axis==='row'?'vertical':'horizontal');
   // Each slot includes its two border pixels as well as nested dividers.
   // A zero flex basis otherwise gives leaf borders extra space on every level.
   const bases=node.children.map(c=>{const count=this.axisCount(c,node.axis);return count*2+(count-1)*6});
   const size=()=>{first.style.flex=`${node.ratio} 1 ${bases[0]}px`;second.style.flex=`${1-node.ratio} 1 ${bases[1]}px`;handle.setAttribute('aria-valuenow',String(Math.round(node.ratio*100)))};size();
   const update=value=>{node.ratio=Math.max(.1,Math.min(.9,value));size();this.workspace.invalidate()};
   let dragging=false;
   handle.onpointerdown=e=>{e.preventDefault();handle.setPointerCapture(e.pointerId);dragging=true};
   handle.onpointermove=e=>{if(!dragging)return;const r=container.getBoundingClientRect(),length=node.axis==='row'?r.width:r.height,offset=node.axis==='row'?e.clientX-r.left:e.clientY-r.top;update((offset-3-bases[0])/Math.max(1,length-6-bases[0]-bases[1]))};
   handle.onpointerup=handle.onpointercancel=()=>{dragging=false;W.save()};
   handle.onkeydown=e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','Home'].includes(e.key))return;e.preventDefault();update(e.key==='Home'?.5:node.ratio+(['ArrowLeft','ArrowUp'].includes(e.key)?-.05:.05));W.save()};
   handle.ondblclick=()=>{update(.5);W.save()};return handle;
  }
  render(node){
   if(node.chart){const config=W.prefs.charts.find(c=>c.id===node.chart),card=this.workspace.createCard(config);card.dataset.minWidth=260;card.dataset.minHeight=160;return card}
   const container=el('div','chart-split '+node.axis),first=this.render(node.children[0]),second=this.render(node.children[1]);
   const a={width:Number(first.dataset.minWidth),height:Number(first.dataset.minHeight)},b={width:Number(second.dataset.minWidth),height:Number(second.dataset.minHeight)};
   const minimum=node.axis==='row'?{width:a.width+b.width+6,height:Math.max(a.height,b.height)}:{width:Math.max(a.width,b.width),height:a.height+b.height+6};
   container.dataset.minWidth=minimum.width;container.dataset.minHeight=minimum.height;container.style.minWidth=minimum.width+'px';container.style.minHeight=minimum.height+'px';
   container.append(first,this.separator(node,container,first,second),second);return container;
  }
  rebuild(){
   this.workspace.interactions.cancel();
   const ids=new Set(W.prefs.charts.map(c=>c.id));for(const id of this.workspace.interactions.ranges.keys())if(!ids.has(id))this.workspace.interactions.ranges.delete(id);
   for(const c of this.workspace.cards.values())c.observer.disconnect();this.workspace.cards.clear();
   this.normalize();$('chart-tabs').replaceChildren();$('chart-grid').replaceChildren();
   for(const tab of W.prefs.tabs){
    const item=el('div','chart-tab'),button=el('button','chart-tab-name',tab.title),close=el('button','chart-tab-close');
    item.dataset.tabId=tab.id;button.role='tab';button.id='tab-'+tab.id;button.setAttribute('aria-controls','tab-panel-'+tab.id);button.title='双击重命名 · 拖动排序';button.onclick=()=>this.activate(tab.id);button.ondblclick=()=>this.rename(tab,button);
    button.draggable=true;button.ondragstart=e=>{this.dragTab=tab.id;e.dataTransfer.setData('application/x-rlsim-tab',tab.id)};button.ondragend=()=>this.dragTab=null;
    item.ondragover=e=>{if([...e.dataTransfer.types].includes('application/x-rlsim-tab'))e.preventDefault()};item.ondrop=e=>{const id=e.dataTransfer.getData('application/x-rlsim-tab');if(!id||id===tab.id)return;e.preventDefault();const from=W.prefs.tabs.findIndex(t=>t.id===id),to=W.prefs.tabs.indexOf(tab);if(from<0)return;W.prefs.tabs.splice(to,0,...W.prefs.tabs.splice(from,1));W.save();this.rebuild()};
    close.append(W.icon('x'));close.setAttribute('aria-label','关闭 Tab '+tab.title);close.title='关闭 Tab';close.onclick=()=>this.closeTab(tab.id);item.append(button,close);$('chart-tabs').append(item);
    const page=el('div','chart-tab-page');page.dataset.tabId=tab.id;page.id='tab-panel-'+tab.id;page.role='tabpanel';page.setAttribute('aria-labelledby',button.id);page.append(this.render(tab.layout));$('chart-grid').append(page);
    button.onkeydown=e=>{const tabs=W.prefs.tabs,index=tabs.indexOf(tab);if(!['ArrowLeft','ArrowRight','Home','End'].includes(e.key))return;e.preventDefault();const next=tabs[e.key==='Home'?0:e.key==='End'?tabs.length-1:(index+(e.key==='ArrowRight'?1:-1)+tabs.length)%tabs.length];this.activate(next.id);$('tab-'+next.id).focus()};
   }
   this.workspace.targets();$('charts-empty').hidden=!!this.workspace.cards.size;
   $('chart-grid-toggle').checked=!!W.prefs.chartGrid;
   if(W.prefs.activeTab)this.activate(W.prefs.activeTab);else W.save();
   this.workspace.invalidate();
  }
 }
 W.ChartLayout=ChartLayout;
})();
