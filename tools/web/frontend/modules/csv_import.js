// Streaming CSV analysis. Imported samples never enter the live telemetry store.
(() => {
 'use strict';
 const W=Workbench,MAX_BYTES=64*1024*1024,MAX_ROWS=20000,MAX_CELLS=2000000;
 const numeric=value=>{const s=String(value??'').trim();if(!s)return null;if(s==='true')return 1;if(s==='false')return 0;const n=Number(s);return Number.isFinite(n)?n:null};
 const body={};
 for(const [field,axes,unit,ids] of [['position','xyz','m',['x','y','z']],['linear_velocity','xyz','m_s',['vx','vy','vz']],['angular_velocity','xyz','rad_s',['wx','wy','wz']]])
  [...axes].forEach((axis,i)=>body[`body.${field}.${axis}_${unit}`]='body.'+ids[i]);
 const jointUnits={q:'rad',target_q:'rad',dq:'rad/s',target_dq:'rad/s',effort:'Nm',tau_ff:'Nm',kp:'Nm/rad',kd:'Nm·s/rad'};
 const jointLabels={q:'实际位置',target_q:'目标位置',dq:'实际速度',target_dq:'目标速度',effort:'估计力矩',tau_ff:'前馈力矩',kp:'策略 Kp',kd:'策略 Kd'};
 function schemaFor(headers){
  if(headers.length>512||new Set(headers).size!==headers.length||headers.some(h=>!h))throw new Error('CSV 表头为空、重复或超过 512 列。');
  const indices=new Map(headers.map((h,i)=>[h,i]));
  if(!indices.has('simulation_time_s')&&!indices.has('received_time_unix_s'))throw new Error('请选择本项目录制或导出的 CSV，需包含 simulation_time_s 或 received_time_unix_s。');
  const names=[...new Set(headers.map(h=>h.match(/^(?:joint|joint_state)\.(.+)\.(?:target_q|target_dq|tau_ff|effort|kp|kd|q|dq)_(?:rad_s|Nm_s_rad|Nm_rad|rad|Nm)$/)?.[1]||h.match(/^(?:joint|joint_state)\.(.+)\.(?:target_q|target_dq|tau_ff|effort|kp|kd|q|dq) \[/)?.[1]).filter(Boolean))];
  const actions=headers.map(h=>h.match(/^action(\d+)$/)||h.match(/^policy\.action\.(\d+) \[/)).filter(Boolean).map(m=>Number(m[1]));
  const basis=new W.TelemetryStore({notify:false});
  basis.configure({robot:{joints:names},policy:{joints:names,actions:{count:Math.min(512,Math.max(-1,...actions)+1)}}},false);
  const base=new Map(basis.descriptors.map(d=>[d.id,d])),descriptors=new Map(),mapping=[];
  const add=(id,label,unit,group,frame,path=[],extra={})=>{
   if(!descriptors.has(id))descriptors.set(id,{...(base.get(id)||{id,label,unit,group,frame,path}),...extra});
   return id;
  };
  const jointSignal=(source,name,key,unit)=>{
   const reference=base.get(`joint.${name}.${['kp','kd'].includes(key)?'q':key}`),category=['kp','kd'].includes(key)?'策略 PD':reference?.path?.at(-1)||'关节数据';
   return add(`${source}.${name}.${key}`,`${name} · ${jointLabels[key]}${source==='joint_state'?' · JointState':''}`,unit||jointUnits[key],source==='joint'?'关节状态':'关节集合（JointState）','关节坐标系',[reference?.path?.[0]||'其他关节',category],
    {joint:name,shortLabel:`${name.replace(/^(FL|FR|RL|RR|HL|HR)[_.-]/i,'')} · ${jointLabels[key]}${source==='joint_state'?' · JointState':''}`});
  };
  for(const [index,header] of headers.entries()){
   if(['simulation_time_s','received_time_unix_s','message_time_s','segment'].includes(header))continue;
   let id,match=header.match(/^(.+) \[([^\]]*)\]$/);
   if(match){
    const [,raw,unit]=match,joint=raw.match(/^(joint|joint_state)\.(.+)\.(target_q|target_dq|tau_ff|effort|kp|kd|q|dq)$/);
    id=joint?jointSignal(joint[1],joint[2],joint[3],unit):add(raw,raw,unit,'其他数值','CSV 原始数据',[],raw.startsWith('command.')?{label:'CSV 指令 '+raw.split('.').at(-1),frame:'机体系；指令来源未记录'}:{});
   }
   else if(body[header]){id=body[header];add(id,id,'','机体状态','世界坐标系');}
   else if((match=header.match(/^(joint|joint_state)\.(.+)\.(target_q|target_dq|tau_ff|effort|kp|kd|q|dq)_(rad_s|Nm_s_rad|Nm_rad|rad|Nm)$/))){
    const [,source,name,key]=match;id=jointSignal(source,name,key);
   }else if((match=header.match(/^action(\d+)$/))){id='policy.action.'+match[1];add(id,'action'+match[1],'','策略诊断','模型输出',['action']);}
   else if(header==='controller.inference_ms'){id='performance.inference';add(id,'推理与解码','ms','运行性能','墙钟耗时',['推理耗时']);}
   else if((match=header.match(/^command\.(vx|vy|wz)_(m_s|rad_s)$/))){id='command.'+match[1];add(id,'ROS 指令 '+match[1],match[1]==='wz'?'rad/s':'m/s','控制指令','机体系；ROS /cmd_vel 中收到的指令，非控制器接受值',[match[1]==='wz'?'转向速度':'平移速度'],{label:'ROS 指令 '+match[1],frame:'机体系；ROS /cmd_vel 中收到的指令，非控制器接受值'});}
   else if((match=header.match(/^body\.quaternion\.([wxyz])$/))){id=add(header,'四元数 '+match[1],'','机体状态','姿态四元数，世界参考',['四元数']);}
   else if(header.endsWith('_received_time_unix_s')){id=add('record.'+header,header,'s','时间与采集','Unix 时间',['信号接收时间']);}
   else if(header==='controller.fsm_state'||header==='controller.reset_robot_mode'){continue;}
   else if(header.startsWith('controller.')){id=add(header,header.replace('controller.',''),header.endsWith('inference_ms')?'ms':'','策略诊断','控制器遥测',['运行状态']);}
   else {id=add('csv.'+header,header,'','其他数值','CSV 原始列');}
   mapping.push({index,id});
  }
  const quaternion=['w','x','y','z'].every(axis=>descriptors.has('body.quaternion.'+axis));
  if(quaternion)for(const axis of ['roll','pitch','yaw'])add('body.'+axis,axis,'rad','机体状态','四元数计算',['姿态']);
  for(const key of ['performance.rtf','performance.policy_hz']){
   if(key==='performance.policy_hz'&&!indices.has('controller.policy_steps'))continue;
   add(key,key,'','运行性能','时间差分计算');
  }
  const order=['机体状态','关节状态','关节集合（JointState）','控制指令','策略诊断','运行性能','时间与采集','其他数值'];
  return {indices,mapping,descriptors:[...descriptors.values()].sort((a,b)=>order.indexOf(a.group)-order.indexOf(b.group)),quaternion,names};
 }

 class RecordingStore{
  constructor(schema){Object.assign(this,{schema,descriptors:schema.descriptors,frames:[],segments:[],byEpoch:new Map(),epoch:0,timeBasis:null,skipped:0,missing:0,previous:null});}
  descriptor(id){return this.descriptors.find(d=>d.id===id)}
  currentEpoch(){return this.byEpoch.get(this.epoch)||[]}
  append(row){
   const {schema}=this,get=key=>numeric(row[schema.indices.get(key)]),wall=get('received_time_unix_s'),sim=get('simulation_time_s');
   if(this.frames.length===0){this.timeBasis=sim!==null?'simulation':'received';this.origin=wall;}
   const t=this.timeBasis==='simulation'?sim:wall!==null&&this.origin!==null?wall-this.origin:null;
   if(t===null){this.skipped++;return;}
   if(this.frames.length>=MAX_ROWS||(this.frames.length+1)*schema.descriptors.length>MAX_CELLS)throw new Error('录包超过浏览器分析上限（20,000 行 / 200 万数值）。请拆分 CSV 后加载，未截断或替换当前数据。');
   const sourceSegment=get('segment'),previous=this.previous;
   const split=previous&&(t<previous.t||(sourceSegment!==null&&previous.sourceSegment!==null&&sourceSegment!==previous.sourceSegment)||(wall!==null&&previous.wall!==null&&wall-previous.wall>1.25));
   const epoch=previous?previous.epoch+(split?1:0):0,values={};
   for(const {index,id} of schema.mapping){values[id]=numeric(row[index]);if(values[id]===null)this.missing++;}
   if(schema.quaternion){
    const [w,x,y,z]=['w','x','y','z'].map(axis=>values['body.quaternion.'+axis]);
    if([w,x,y,z].every(W.finite)){values['body.roll']=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));values['body.pitch']=Math.asin(Math.max(-1,Math.min(1,2*(w*y-z*x))));values['body.yaw']=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
   }
   if(previous&&!split&&wall!==null&&previous.wall!==null&&wall>previous.wall){
    if(this.timeBasis==='simulation'&&!W.finite(values['performance.rtf']))values['performance.rtf']=(t-previous.t)/(wall-previous.wall);
    const current=get('controller.policy_steps'),last=previous.values['controller.policy_steps'];
    if(current!==null&&W.finite(last)&&current>=last&&!W.finite(values['performance.policy_hz']))values['performance.policy_hz']=(current-last)/(wall-previous.wall);
   }
   const frame={t,wall,epoch,sourceSegment,values,messageTime:get('message_time_s'),simulationTime:sim};
   if(!this.byEpoch.has(epoch)){this.byEpoch.set(epoch,[]);this.segments.push(epoch)}
   this.frames.push(frame);this.byEpoch.get(epoch).push(frame);this.previous=frame;
  }
 }

 async function read(stream,{signal,onProgress=()=>{},bytes=0}={}){
  if(bytes>MAX_BYTES){await stream.cancel().catch(()=>{});throw new Error('CSV 文件超过 64 MB，请拆分后加载。');}
  const reader=stream.getReader(),decoder=new TextDecoder('utf-8',{fatal:true});
  let loaded=0,field='',row=[],quoted=false,afterQuote=false,afterCR=false,headers=null,store=null;
  const abort=()=>reader.cancel().catch(()=>{}),check=()=>{if(signal?.aborted)throw new DOMException('已取消加载','AbortError')};
  signal?.addEventListener('abort',abort,{once:true});
  const finishRow=()=>{
   row.push(field);field='';afterQuote=false;
   if(row.length===1&&!row[0].trim()){row=[];return;}
   if(!headers){headers=row.map((h,i)=>(i===0?h.replace(/^\ufeff/,''):h).trim());store=new RecordingStore(schemaFor(headers));}
   else {if(row.length!==headers.length)throw new Error('CSV 数据列数与表头不一致，请检查文件是否写盘完成。');store.append(row);}
   row=[];
  };
  const consume=text=>{
   for(const c of text){
    if(afterCR){afterCR=false;if(c==='\n')continue;}
    if(quoted){if(c==='"'){quoted=false;afterQuote=true}else field+=c;}
    else if(afterQuote&&c==='"'){field+='"';quoted=true;afterQuote=false;}
    else if(c===','){row.push(field);field='';afterQuote=false;}
    else if(c==='\n'||c==='\r'){finishRow();afterCR=c==='\r';}
    else if(c==='"'&&!field&&!afterQuote){quoted=true;}
    else {if(afterQuote||c==='"')throw new Error('CSV 引号格式错误。');field+=c;}
    if(field.length>8192||row.length>512)throw new Error('CSV 字段或列数超过分析上限。');
   }
  };
  try{
   while(true){check();const chunk=await reader.read();check();if(chunk.done)break;loaded+=chunk.value.byteLength;if(loaded>MAX_BYTES)throw new Error('CSV 文件超过 64 MB，请拆分后加载。');consume(decoder.decode(chunk.value,{stream:true}));onProgress(loaded,bytes);await new Promise(resolve=>setTimeout(resolve,0));}
   consume(decoder.decode());if(quoted)throw new Error('CSV 文件未完整写盘：引号字段尚未结束。');if(field||row.length||afterQuote)finishRow();
   if(!store?.frames.length)throw new Error('录包没有可分析的有效时间与数据行。');
   return store;
  }finally{signal?.removeEventListener('abort',abort);await reader.cancel().catch(()=>{});reader.releaseLock();}
 }
 W.CsvImport={read,MAX_BYTES,MAX_ROWS,MAX_CELLS};
})();
