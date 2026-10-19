// UI preferences and shared session state. Runtime state is never persisted.
(()=>{"use strict";
const $=id=>document.getElementById(id), clone=v=>JSON.parse(JSON.stringify(v)), finite=v=>typeof v==='number'&&Number.isFinite(v), fmt=(v,n=3)=>finite(v)?v.toFixed(n):'—';
function el(tag,className,text){const n=document.createElement(tag);if(className)n.className=className;if(text!==undefined)n.textContent=text;return n;}
function icon(name){const n=document.createElementNS('http://www.w3.org/2000/svg','svg'),u=document.createElementNS(n.namespaceURI,'use');u.setAttribute('href','#i-'+name);n.append(u);return n;}
const defaults={theme:'system',sidebar:false,left:288,right:264,window:30,sampleHz:4,chartGrid:false,charts:[],tabs:[],activeTab:null};
let prefs;try{prefs={...defaults,...JSON.parse(localStorage.getItem('rlsim.workbench.v2')||'{}')}}catch{prefs=clone(defaults)}
if(!RLsimAppearance.valid(prefs.theme))prefs.theme='system';
if(!Array.isArray(prefs.charts))prefs.charts=clone(defaults.charts);
if(![1,2,4,10].includes(prefs.sampleHz))prefs.sampleHz=4;
if(!Array.isArray(prefs.tabs))prefs.tabs=[];
function save(){try{localStorage.setItem('rlsim.workbench.v2',JSON.stringify({theme:prefs.theme,sidebar:!!prefs.sidebar,left:prefs.left,right:prefs.right,window:prefs.window,sampleHz:prefs.sampleHz,chartGrid:!!prefs.chartGrid,charts:prefs.charts,tabs:prefs.tabs,activeTab:prefs.activeTab}))}catch{}}
const state={page:'simulation',boot:null,online:false,selected:'',current:null,detail:null,resolved:null,activeId:'',pending:null,recordBusy:false,mode:'live',events:[],logs:{},runs:[]};
function notice(text,ok=false){$('notice-text').textContent=text;$('notice-text').title=text;$('notice').classList.toggle('success',ok);$('notice').hidden=false;clearTimeout(notice.timer);notice.timer=setTimeout(()=>$('notice').hidden=true,10000)}
function feedback(id,text){const n=$(id);n.textContent=text;n.hidden=!text}
function options(n,entries,chosen){n.replaceChildren(...entries.map(([v,t])=>new Option(t,v)));if(entries.some(([v])=>v===chosen))n.value=chosen;}
function pill(id,text,tone=''){const n=$(id);n.textContent=text;n.className='pill '+tone;}
function signalFresh(name){const e=state.current?.telemetry?.signals?.[name];return !!(state.online&&state.current?.bridge_live&&e&&Date.now()/1000-e.received_at<2)}
function live(){return !!(state.current?.owned&&signalFresh('controller'))}
function canSustainVelocity(){const c=state.current?.telemetry?.signals?.controller?.data;return state.mode==='live'&&live()&&c.controller_running!==false&&!state.current?.controller_unknown&&(!window.Workbench?.controller||Workbench.controller.status().running)&&c.ready&&c.policy_ready&&c.fsm_state==='policy'&&c.simulation_running&&!c.fault}
function canDrive(){return state.page==='simulation'&&canSustainVelocity()}
function angle(v){return prefs.angle==='deg'?v*180/Math.PI:v}function radians(v){return prefs.angle==='deg'?v*Math.PI/180:v}
function theme(){
 document.documentElement.dataset.theme=RLsimAppearance.resolve(prefs.theme);
 for(const id of ['theme','theme-settings'])if($(id))$(id).value=prefs.theme;
 window.Workbench?.charts?.invalidate();window.Workbench?.renderer?.draw();save();
}
function colors(){const css=getComputedStyle(document.documentElement);return name=>css.getPropertyValue(name).trim();}
RLsimAppearance.media.addEventListener('change',()=>{if(prefs.theme==='system')theme()});

async function busy(button,action){if(button.classList.contains('loading'))return;button.classList.add('loading');button.disabled=true;try{return await action()}catch(e){notice(e.message);throw e}finally{button.classList.remove('loading');button.disabled=false;window.Workbench?.renderStatus?.()}}
window.Workbench={$: $,clone,finite,fmt,el,icon,prefs,save,state,notice,feedback,options,pill,signalFresh,live,canSustainVelocity,canDrive,angle,radians,theme,colors,busy};
})();
