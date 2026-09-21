import test from 'node:test';
import assert from 'node:assert/strict';
import {fieldBounds, autoFrame, roleIds, renderDue, placeLabels, TrailBuffer, PoseTrack, robotAppearance, SceneCues} from '../static/js/twin-model.js';

const field = {arena:{x:[-2,2],y:[-2,2]},limo_area:{x:[-5,-3],y:[-3,3]},spare_area:{x:[-3,-2.2],y:[-2,2],pitch:.4}};
const hello={field,drones:['reader','finder'],limos:['carrier'],freshness_s:{pose:1,odom:2}};
const active={mode:'active',phase:'search',signal:'blue'};
const row={kind:'drone',role:'finder',pose:{x:1,y:2,z:1,yaw:0},pose_age:.1};
const state={run:{state:'RUNNING'},mission:{age:0,led:{finder:'red'}}};

test('field bounds follow supplied regions, including renamed and expanded deployment coordinates',()=>{
  assert.deepEqual(fieldBounds(field),{x:[-5,2],y:[-3,3]});
  assert.deepEqual(fieldBounds({arena:{x:[10,14],y:[20,24]},spare_area:{x:[8,9],y:[21,22]}}),{x:[10,14],y:[20,24]});
});
test('visitor role selection comes only from hello and never observes spare telemetry',()=>{
  const fleet={reader:row,finder:row,carrier:{kind:'limo'}};
  Object.defineProperty(fleet,'reserve',{get(){throw Error('spare telemetry accessed');}});
  assert.deepEqual(roleIds(hello,fleet),['reader','finder','carrier']);
  assert.deepEqual(roleIds({drones:['new-a'],limos:['new-b','new-c']},fleet),['new-a','new-b','new-c']);
});
test('a high refresh rate display cannot request more than sixty rendered frames per second',()=>{
  let last=null,count=0;
  for(let i=0;i<1440;i++) {
    const now=i*1000/144;
    if(renderDue(now,last)) {if(last!==null)assert.ok(now-last>=1000/60-1e-7);last=now;count++;}
  }
  assert.ok(count<=601&&count>=400);
});
test('labels colliding at the same projected pose occupy separate stable rows without moving their anchors',()=>{
  const input=Array.from({length:6},(_,i)=>({id:`role-${i}`,x:200,y:200,width:100,height:22}));
  const copy=structuredClone(input),result=placeLabels(input,{width:500,height:300});
  assert.deepEqual(input,copy);
  assert.deepEqual(result.map(r=>r.id),input.map(r=>r.id));
  assert.deepEqual(placeLabels(input,{width:500,height:300}),result);
  for(let i=0;i<result.length;i++)for(let j=0;j<i;j++) {
    const a=result[i],b=result[j];
    assert.ok(Math.abs(a.x-b.x)>=(a.width+b.width)/2||Math.abs(a.y-b.y)>=(a.height+b.height)/2);
  }
});
test('labels near every frame edge remain within the scene above the reserved narrative',()=>{
  const items=[{id:'a',x:-5,y:-5,width:90,height:24},{id:'b',x:0,y:0,width:90,height:24},
    {id:'c',x:350,y:140,width:90,height:24},{id:'d',x:350,y:140,width:90,height:24}];
  for(const label of placeLabels(items,{width:350,height:140})) {
    assert.ok(label.x-label.width/2>=0&&label.x+label.width/2<=350);
    assert.ok(label.y-label.height/2>=0&&label.y+label.height/2<=140);
  }
});
test('diagonal automatic framing includes every floor and altitude corner at portrait and wide aspects',()=>{
  for(const aspect of [.45,1,2.7]) for(const azimuth of [.2,Math.PI/4,1.5]) {
    const frame=autoFrame(field,aspect,{height:2.4,azimuth});
    for(const x of [-5,2]) for(const y of [-3,3]) for(const z of [0,2.4]) {
      const point=[x-frame.target[0],z-frame.target[1],-y-frame.target[2]];
      const dot=v=>v.reduce((sum,n,i)=>sum+n*point[i],0);
      assert.ok(Math.abs(dot(frame.right))<frame.halfWidth);
      assert.ok(Math.abs(dot(frame.up))<frame.halfHeight);
    }
  }
});
test('trail memory remains fixed over a simulated 30 minute stream and expires after ten seconds',()=>{
  const trail=new TrailBuffer(128,10),storage=trail.positions,timestamps=trail.times;
  for(let i=0;i<18000;i++) trail.push(i/10,{x:i,y:1,z:2});
  assert.equal(trail.positions,storage);assert.equal(trail.times,timestamps);
  assert.equal(trail.length,128);
  const positions=new Float32Array(128*6),times=new Float32Array(128*2);
  const count=trail.writeSegments(positions,times,1799.9);
  assert.ok(count>0&&count<=200);
  assert.equal(trail.writeSegments(positions,times,1811),0);
});
test('wrapped trail emits chronological adjacent segments without joining the newest to oldest',()=>{
  const trail=new TrailBuffer(4,10);
  for(let i=0;i<8;i++) trail.push(i,{x:i,y:0,z:1});
  const positions=new Float32Array(24),times=new Float32Array(8);
  assert.equal(trail.writeSegments(positions,times,7),6);
  assert.deepEqual([positions[0],positions[3],positions[6],positions[9],positions[12],positions[15]],[4,5,5,6,6,7]);
  assert.deepEqual([...times.slice(0,6)],[4,5,5,6,6,7]);
});
test('pose interpolation starts immediately, reaches the new sample and takes the short yaw arc',()=>{
  const track=new PoseTrack({x:0,y:0,z:0,yaw:Math.PI-.1});
  track.update({x:2,y:4,z:1,yaw:-Math.PI+.1},10);
  const middle=track.sample(10.05);
  assert.ok(Math.abs(middle.x-1)<1e-6);assert.ok(Math.abs(middle.y-2)<1e-6);
  assert.ok(Math.abs(Math.abs(middle.yaw)-Math.PI)<1e-6);
  assert.equal(track.sample(11).x,2);
});
test('an invalid or missing pose never erases the last received position',()=>{
  const track=new PoseTrack({x:2,y:3,z:1,yaw:0});
  assert.equal(track.update(null,1),false);
  assert.equal(track.update({x:NaN,y:0,z:0},1),false);
  assert.deepEqual(track.sample(3),{x:2,y:3,z:1,yaw:0});
});
test('freshness ages between packets, removes altitude stems, and suppresses stale LED signals',()=>{
  assert.deepEqual(robotAppearance(hello,'finder',row,state,active,0),{fresh:true,opacity:1,stem:true,signal:'red'});
  const stale=robotAppearance(hello,'finder',row,state,active,1);
  assert.equal(stale.fresh,false);assert.equal(stale.stem,false);assert.equal(stale.signal,'off');
  assert.ok(stale.opacity<.5);
});
test('waiting for mission data preserves fresh pose opacity and altitude stems independently of connection loss',()=>{
  for(const mission of [null,{age:3.5,led:{finder:'red'}}]) {
    const waiting=robotAppearance(hello,'finder',row,{...state,mission},{...active,mode:'waiting'},0);
    assert.deepEqual(waiting,{fresh:true,opacity:1,stem:true,signal:'off'});
    const offline=robotAppearance(hello,'finder',row,{...state,mission},{...active,mode:'offline'},0);
    assert.equal(offline.fresh,true,'pose freshness remains independent from connection state');
    assert.equal(offline.opacity,.28);assert.equal(offline.stem,false);assert.equal(offline.signal,'off');
  }
});
test('run and connection priority ignore mission LEDs while DONE can retain the last mission',()=>{
  for(const mode of ['idle','offline','waiting','paused']) assert.equal(robotAppearance(hello,'finder',row,state,{...active,mode},0).signal,'off');
  for(const run of ['LANDING','ABORTED']) assert.equal(robotAppearance(hello,'finder',row,{...state,run:{state:run}},active,0).signal,'off');
  assert.equal(robotAppearance(hello,'finder',row,{run:{state:'DONE'},mission:{age:99,led:{finder:'green'}}},active,0).signal,'green');
});
test('an assigned role with a missing pose remains translucent without an altitude stem',()=>{
  const appearance=robotAppearance(hello,'finder',{kind:'drone',role:'finder',pose:null,pose_age:null},state,active,0);
  assert.equal(appearance.stem,false);assert.ok(appearance.opacity<.5);
});
test('discovery ring and return floor wash fire once and reset only for a new run',()=>{
  const cues=new SceneCues();
  const found={run:{state:'RUNNING',since:1},mission:{P_N:{x:2,y:3,z:1},finder:'finder'}};
  assert.equal(cues.update(found,active,1).found,true);
  assert.equal(cues.update(found,active,2).found,false);
  assert.equal(cues.update(found,{...active,phase:'return'},3).wash,true);
  assert.equal(cues.update(found,{...active,phase:'done'},4).wash,false);
  assert.equal(cues.update(found,{...active,mode:'paused'},5).visible,false);
  cues.update({run:{state:'IDLE'},mission:null},{mode:'idle'},6);
  assert.equal(cues.update({...found,run:{state:'RUNNING',since:7}},active,7).found,true);
});
