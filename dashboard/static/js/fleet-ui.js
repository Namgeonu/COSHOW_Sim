import {fixed} from './view-model.js';
const node=(tag,text,className)=>{const value=document.createElement(tag);if(text!=null)value.textContent=text;if(className)value.className=className;return value;};
export function createFleetUI({onSettings}){
  const panel=document.querySelector('#stack-panel');panel.hidden=false;
  const top=node('div',null,'stack-summary'),status=node('p'),buttons=node('div',null,'fleet-buttons');
  const start=node('button','스택 기동'),restart=node('button','스택 재기동'),stop=node('button','스택 정지');buttons.append(start,restart,stop);top.append(status,buttons);
  const editor=node('details'),summary=node('summary','기체 교체 설정');editor.id='fleet-editor';
  const hint=node('p','기본 화면에는 역할 기체만 표시합니다. 교체할 때만 인벤토리를 펼쳐 배정하세요.','label');
  const table=node('table'),head=node('thead'),header=node('tr');
  for(const title of ['기체','배정 역할','링크 age','배터리','RSSI','무장 · 자세','AI Deck / 차량 ping','카메라'])header.append(node('th',title));head.append(header);
  const body=node('tbody');table.append(head,body);const scroll=node('div',null,'table-scroll');scroll.append(table);
  const recommend=node('button','추천 로스터'),save=node('button','배정 검토'),actions=node('div',null,'fleet-buttons');actions.append(recommend,save);
  const notice=node('p','LIMO 교체는 Jetson에서 역할 네임스페이스로 재기동한 뒤 설정을 확인합니다.','label');
  const response=node('p','', 'label');response.setAttribute('role','status');
  editor.append(summary,hint,scroll,actions,notice);panel.append(top,editor,response);
  const dialog=node('dialog'),heading=node('h2','역할 배정 확인'),changes=node('pre'),confirm=node('button','배정 저장'),cancel=node('button','취소');
  dialog.append(heading,changes,confirm,cancel);panel.append(dialog);
  let settings=null,hello=null,state=null,connected=false,busy=false,draft={},rows=new Map(),signature='';
  let settingsFresh=false,loadGeneration=0;
  const canEdit=()=>Boolean(settingsFresh&&settings&&hello&&connected&&state?.run?.state==='IDLE'&&!busy&&!state.stack?.busy);
  const canStop=()=>canEdit()&&![state?.stack?.crazyflie_server,state?.stack?.aideck].includes('external');
  const completeDraft=()=>Boolean(hello?.drones.every(role=>draft[role]));
  async function request(action,method='POST',payload){
    if(!canEdit())return null;
    busy=true;render();response.textContent='처리 중';
    try{const result=await fetch('/api/fleet/'+action,{method,headers:{'Content-Type':'application/json'},...(payload?{body:JSON.stringify(payload)}:{})});
      const value=await result.json();if(!result.ok||!value.ok)throw Error(value.error||'요청 거부');
      response.textContent='완료';return value.result;
    }catch(error){response.textContent=error.message;return null;}
    finally{busy=false;render();}
  }
  start.addEventListener('click',()=>request('start'));restart.addEventListener('click',()=>request('restart'));
  stop.addEventListener('click',()=>canStop()?request('stop'):null);
  recommend.addEventListener('click',async()=>{
    const generation=loadGeneration,result=await request('recommend','GET');
    if(result&&generation===loadGeneration){draft=result;refreshSelectors();response.textContent='추천 배정입니다. 검토 후 저장하세요.';}
  });
  save.addEventListener('click',()=>{
    if(!canEdit()||!completeDraft())return;
    changes.textContent=(hello?.drones||[]).map(role=>`${hello.display_names?.[role]||role} (${role})  ${settings.roster[role]||'미배정'} → ${draft[role]||'미배정'}`).join('\n');
    dialog.showModal();
  });
  cancel.addEventListener('click',()=>dialog.close());
  confirm.addEventListener('click',async()=>{
    if(!dialog.open||!canEdit()||!completeDraft())return;
    dialog.close();const result=await request('roster','PUT',{roster:draft,expected_hash:settings.roster_hash});
    if(result){await load();response.textContent='배정 저장 완료 · 스택 재기동 후 점검하세요.';}
  });
  function refreshSelectors(){
    for(const [id,row] of rows)if(row.select)row.select.value=Object.keys(draft).find(role=>draft[role]===id)||'';
    render();
  }
  function build(){
    if(!settings||!hello)return;
    const next=JSON.stringify([settings.fleet,settings.roster,hello.drones]);if(next===signature)return;signature=next;
    draft={...settings.roster};rows=new Map();body.replaceChildren();
    const all=[...(settings.fleet.drones||[]).map(item=>({...item,kind:'drone'})),...(settings.fleet.limos||[]).map(item=>({...item,kind:'limo'}))];
    all.sort((a,b)=>Number(Object.values(draft).includes(b.id)||hello.limos.includes(b.namespace))-Number(Object.values(draft).includes(a.id)||hello.limos.includes(a.namespace)));
    for(const item of all){
      const tr=node('tr');tr.dataset.fleetId=item.id;tr.append(node('td',item.id));const role=node('td');let select=null;
      if(item.kind==='drone'){
        select=node('select');select.setAttribute('aria-label',`${item.id} 역할`);select.append(new Option('미배정',''));
        for(const id of hello.drones)select.append(new Option(`${hello.display_names?.[id]||id} (${id})`,id));
        select.addEventListener('change',()=>{for(const id of Object.keys(draft))if(draft[id]===item.id)delete draft[id];if(select.value)draft[select.value]=item.id;refreshSelectors();});role.append(select);
      }else role.textContent=hello.display_names?.[item.namespace]||item.namespace;
      tr.append(role);const cells=Array.from({length:6},()=>node('td','—'));tr.append(...cells);body.append(tr);rows.set(item.id,{tr,select,cells});
    }
    refreshSelectors();
  }
  function render(){
    const stack=state?.stack||{},allowed=canEdit();
    start.disabled=!allowed||[stack.crazyflie_server,stack.aideck].some(value=>value!=='down');
    restart.disabled=!allowed||[stack.crazyflie_server,stack.aideck].includes('external');
    stop.disabled=!canStop();
    recommend.disabled=!allowed;save.disabled=!allowed||!completeDraft();confirm.disabled=!allowed||!completeDraft();
    if(!allowed)dialog.close();
    for(const row of rows.values())if(row.select)row.select.disabled=!allowed;
    if(!state)return;
    const labels={up:'실행 중',down:'종료',external:'외부 실행'};
    const match=stack.roster_hash&&stack.roster_hash===stack.applied_hash;
    const armedSpares=Object.values(state.robots||{}).filter(robot=>robot.role==null&&robot.armed===true).length;
    status.textContent=`드론 서버 ${labels[stack.crazyflie_server]||'확인 중'} · 카메라 ${labels[stack.aideck]||'확인 중'} · ${match?'배정 일치':stack.applied_hash?'재기동 필요':'적용 배정 미확인'} · 라디오 ${stack.radios??'—'}개${armedSpares?' · 미배정 기체 무장 '+armedSpares+'대':''}`;
    for(const [physical,row] of rows){
      const robot=Object.values(state.robots||{}).find(value=>value.fleet_id===physical)||{};
      const freshAge=robot.kind==='drone'?robot.status_age:robot.pose_age;
      const values=[`${fixed(freshAge,2)} s`,`${fixed(robot.battery_v,2)} V`,fixed(robot.rssi,0),
        `${robot.armed===true?'무장':robot.armed===false?'해제':'—'} · ${robot.tumbled===true?'넘어짐':robot.tumbled===false?'정상':'—'}`,
        robot.ping?.ok===true?`${fixed(robot.ping.rtt_ms,1)} ms`:robot.ping?.ok===false?'응답 없음':'정보 없음',
        robot.role&&robot.kind==='drone'?`${fixed(robot.camera?.fps,1)} fps`:'—'];
      row.cells.forEach((cell,index)=>{if(cell.textContent!==values[index])cell.textContent=values[index];});
      row.tr.classList.toggle('spare-armed',robot.role==null&&robot.armed===true);
    }
  }
  async function load(){
    const generation=++loadGeneration;settingsFresh=false;dialog.close();render();
    try{
      const result=await fetch('/api/admin');
      if(!result.ok)throw Error('설정 수신 실패');
      const value=await result.json();
      if(generation!==loadGeneration)return false;
      settings=value;settingsFresh=true;build();onSettings(settings);render();return true;
    }catch{
      if(generation===loadGeneration){settingsFresh=false;response.textContent='배정 설정 수신 대기';render();}
      return false;
    }
  }
  load();
  return {update(nextHello,nextState,isConnected){
    if(connected&&!isConnected){settingsFresh=false;loadGeneration++;dialog.close();}
    hello=nextHello;state=nextState;connected=isConnected;build();render();
  },load};
}
