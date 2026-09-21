import test from 'node:test';
import assert from 'node:assert/strict';
import {createFleetUI} from '../static/js/fleet-ui.js';

const tick=()=>new Promise(resolve=>setImmediate(resolve));
const settings=(physical='CF1')=>({fleet:{drones:[{id:'CF1'},{id:'CF2'},{id:'CF3'}],limos:[]},roster:{one:physical,two:'CF2'},roster_hash:'revision-'+physical});
const hello={drones:['one','two'],limos:[],display_names:{}};
const snapshot=(physical='CF1',run='IDLE')=>({run:{state:run},stack:{crazyflie_server:'down',aideck:'down'},
  robots:{one:{fleet_id:physical,kind:'drone',role:'one'},two:{fleet_id:'CF2',kind:'drone',role:'two'}}});

function fixture(t){
  const nodes=[],requests=[],loaded=[];
  class Element {
    constructor(tag,text=''){Object.assign(this,{tag,textContent:text,children:[],dataset:{},events:{},disabled:false,open:false});this.classList={toggle(){}};nodes.push(this);}
    append(...children){this.children.push(...children);} replaceChildren(...children){this.children=children;}
    setAttribute(){} addEventListener(type,callback){this.events[type]=callback;}
    showModal(){this.open=true;} close(){this.open=false;}
    click(){if(!this.disabled)return this.events.click?.();}
  }
  const panel=new Element('panel');
  const originals=Object.fromEntries(['document','Option','fetch'].map(key=>[key,Object.getOwnPropertyDescriptor(globalThis,key)]));
  const replacements={document:{querySelector:()=>panel,createElement:tag=>new Element(tag)},
    Option:class extends Element{constructor(text,value){super('option',text);this.value=value;}},
    fetch:(url,options)=>new Promise((resolve,reject)=>requests.push({url,options,resolve,reject}))};
  for(const [key,value] of Object.entries(replacements))Object.defineProperty(globalThis,key,{value,configurable:true,writable:true});
  t.after(()=>{for(const [key,value] of Object.entries(originals)){if(value)Object.defineProperty(globalThis,key,value);else delete globalThis[key];}});
  const ui=createFleetUI({onSettings:value=>loaded.push(value)});
  const button=text=>nodes.find(node=>node.tag==='button'&&node.textContent===text);
  const select=id=>nodes.findLast(node=>node.tag==='tr'&&node.dataset.fleetId===id)?.children[1].children[0];
  const respond=async(index,value,ok=true)=>{requests[index].resolve({ok,json:async()=>value});await tick();};
  return {ui,nodes,requests,loaded,button,select,respond,dialog:nodes.find(node=>node.tag==='dialog')};
}

test('fleet writes stay locked until fresh settings load, including initial state delivery',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);
  assert.equal(f.button('스택 기동').disabled,true);
  assert.equal(f.button('추천 로스터').disabled,true);
  await f.respond(0,settings());
  assert.equal(f.button('스택 기동').disabled,false);
  assert.equal(f.button('배정 검토').disabled,false);
  assert.equal(f.nodes.find(node=>node.tag==='details').open,false);
});

test('explicit stack stop requires fresh settings, IDLE, connection and no active fleet operation',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);
  assert.equal(f.button('스택 정지').disabled,true);
  await f.respond(0,settings());
  assert.equal(f.button('스택 정지').disabled,false);
  for(const run of ['CHECKING','READY','RUNNING','LANDING','DONE','ABORTED']) {
    f.ui.update(hello,snapshot('CF1',run),true);
    assert.equal(f.button('스택 정지').disabled,true,run);
    await f.button('스택 정지').events.click();
    assert.equal(f.requests.length,1,'a queued stop event must not bypass the run guard');
  }
  f.ui.update(hello,{...snapshot(),stack:{crazyflie_server:'up',aideck:'up',busy:true}},true);
  assert.equal(f.button('스택 정지').disabled,true);
  await f.button('스택 정지').events.click();assert.equal(f.requests.length,1);
  f.ui.update(hello,snapshot(),false);
  assert.equal(f.button('스택 정지').disabled,true);
  await f.button('스택 정지').events.click();assert.equal(f.requests.length,1);
});

test('explicit stack stop posts once, blocks concurrent actions, then allows a new start from received down state',async t=>{
  const f=fixture(t),up={...snapshot(),stack:{crazyflie_server:'up',aideck:'up'}};
  f.ui.update(hello,up,true);await f.respond(0,settings());
  assert.equal(f.button('스택 정지').disabled,false);
  f.button('스택 정지').click();
  assert.equal(f.requests[1].url,'/api/fleet/stop');assert.equal(f.requests[1].options.method,'POST');
  assert.equal(f.button('스택 정지').disabled,true);assert.equal(f.button('스택 재기동').disabled,true);
  await f.button('스택 정지').events.click();assert.equal(f.requests.length,2);
  f.ui.update(hello,snapshot(),true);
  await f.respond(1,{ok:true,result:{crazyflie_server:'down',aideck:'down',applied_hash:null}});
  assert.equal(f.button('스택 기동').disabled,false);
  f.button('스택 기동').click();assert.equal(f.requests[2].url,'/api/fleet/start');
  await f.respond(2,{ok:true,result:up.stack});
});

test('an external stack cannot be stopped from the UI or a queued click handler',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);await f.respond(0,settings());
  f.ui.update(hello,{...snapshot(),stack:{crazyflie_server:'up',aideck:'external'}},true);
  assert.equal(f.button('스택 정지').disabled,true);
  await f.button('스택 정지').events.click();assert.equal(f.requests.length,1);
});

test('reconnect reload discards the old roster confirmation and uses the new physical assignment',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);await f.respond(0,settings());
  f.button('배정 검토').click();assert.equal(f.dialog.open,true);
  f.ui.update(hello,snapshot('CF3'),false);
  assert.equal(f.dialog.open,false,'disconnect invalidates a previously reviewed assignment');
  f.ui.load();f.ui.update(hello,snapshot('CF3'),true);
  assert.equal(f.button('배정 검토').disabled,true);
  await f.respond(1,settings('CF3'));
  assert.equal(f.select('CF1').value,'');assert.equal(f.select('CF3').value,'one');
  f.button('배정 검토').click();
  assert.match(f.nodes.find(node=>node.tag==='pre').textContent,/one \(one\)  CF3 → CF3/);
  f.button('배정 저장').click();
  assert.deepEqual(JSON.parse(f.requests[2].options.body),{roster:settings('CF3').roster,expected_hash:settings('CF3').roster_hash});
  await f.respond(2,{ok:false,error:'fixture completes request'},false);
});

test('an older settings response cannot overwrite a newer hello reload',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot('CF3'),true);
  f.ui.load();await f.respond(1,settings('CF3'));await f.respond(0,settings());
  assert.deepEqual(f.loaded.map(value=>value.roster.one),['CF3']);
  assert.equal(f.select('CF3').value,'one');assert.equal(f.select('CF1').value,'');
});

test('failed settings refresh keeps writes locked and late events cannot submit outside IDLE',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);await f.respond(0,settings());
  f.button('배정 검토').click();f.ui.load();
  assert.equal(f.dialog.open,false);
  await f.respond(1,{error:'unavailable'},false);
  assert.equal(f.button('배정 저장').disabled,true);assert.equal(f.button('스택 재기동').disabled,true);
  // A queued input event must also re-check the current state, independently of disabled styling.
  f.ui.update(hello,snapshot('CF1','RUNNING'),true);
  await f.button('배정 저장').events.click();
  f.button('배정 검토').events.click();
  assert.equal(f.dialog.open,false);assert.equal(f.requests.length,2);
});

test('leaving IDLE closes an open review and rejects a queued confirmation event',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);await f.respond(0,settings());
  f.button('배정 검토').click();assert.equal(f.dialog.open,true);
  f.ui.update(hello,snapshot('CF1','CHECKING'),true);
  assert.equal(f.dialog.open,false);assert.equal(f.button('배정 저장').disabled,true);
  await f.button('배정 저장').events.click();assert.equal(f.requests.length,1);
});

test('a recommendation from before reconnect cannot replace the freshly loaded roster draft',async t=>{
  const f=fixture(t);f.ui.update(hello,snapshot(),true);await f.respond(0,settings());
  f.button('추천 로스터').click();
  f.ui.update(hello,snapshot('CF3'),false);f.ui.load();f.ui.update(hello,snapshot('CF3'),true);
  await f.respond(2,settings('CF3'));
  await f.respond(1,{ok:true,result:settings().roster});
  assert.equal(f.select('CF3').value,'one');assert.equal(f.select('CF1').value,'');
});
