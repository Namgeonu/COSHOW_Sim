import test from 'node:test';
import assert from 'node:assert/strict';
import { present, elapsed, interests, makeNames, navigationGoal } from '../static/js/view-model.js';

const hello = {drones:['reader','one','two','three'],limos:['carrier','rescuer'],
  observe_drone:'reader',mission_marker_ids:[11,14],marker_id_offset:10,
  display_names:{reader:'판독 드론',one:'탐색 드론 B',two:'탐색 드론 C',three:'탐색 드론 D'},
  lanes:{one:[],two:[],three:[]},idle_lines:['소개 하나','소개 둘','소개 셋']};
const state = () => ({run:{state:'RUNNING',external_bt:false,elapsed_s:35.4},
  mission:{phase:'search',age:.2,target_id:3,led:{one:'blue'}},robots:{}});

test('run overrides phase, external observation bypasses only idle guard',()=>{
  const value=state(); value.run.state='IDLE';
  assert.equal(present(hello,value).mode,'idle');
  assert.equal(present(hello,value).signal,'off');
  value.run.external_bt=true;
  assert.equal(present(hello,value).mode,'active');
  value.run.state='LANDING';
  assert.equal(present(hello,value).sentence,'잠시 후 다시 시작합니다');
  assert.equal(present(hello,value).signal,'off');
});
test('stale mission and disconnect clear phase signal; DONE keeps completion',()=>{
  const value=state(); value.mission.age=9;
  assert.equal(present(hello,value).mode,'waiting');
  value.run.state='DONE'; value.mission.phase='done';
  assert.equal(present(hello,value).sentence,'구조 완료');
  assert.equal(present(hello,value,{connected:false}).mode,'offline');
});
test('labels count configured searchers and phase color comes only from led',()=>{
  const value=state();
  assert.equal(present(hello,value).sentence,'탐색 드론 3대가 3번 조난자를 찾고 있습니다');
  assert.equal(present(hello,value).signal,'blue');
  value.mission.led={};
  assert.equal(present(hello,value).signal,'off');
  const added={...hello,lanes:{...hello.lanes,four:[]}};
  assert.match(present(added,value).sentence,/탐색 드론 4대/);
});
test('finder uses display name and interest badges exclude unrelated detections',()=>{
  const value=state(); value.mission.phase='capture';value.mission.finder='one';
  assert.match(present(hello,value).sentence,/탐색 드론 B가/);
  assert.deepEqual(interests(hello,value,'one',[1,2,3]),[3]);
  assert.deepEqual(interests(hello,value,'reader',[1,13]),[13]);
  assert.deepEqual(interests(hello,value,'reader',[1,2]),[]);
});
test('observer and marker range follow hello even when lanes and IDs change',()=>{
  const changed={...hello,observe_drone:'two',mission_marker_ids:[41,44],marker_id_offset:40};
  const value=state();
  assert.deepEqual(interests(changed,value,'two',[13,41,43,45]),[41,43]);
  assert.deepEqual(interests(changed,value,'reader',[13,3]),[3]);
  assert.deepEqual(interests({...changed,mission_marker_ids:undefined},value,'two',[41]),[]);
});
test('elapsed uses server seconds, unknown data never becomes zero',()=>{
  assert.equal(elapsed(65.99),'01:05');
  assert.equal(elapsed(undefined),'—:—');
  assert.equal(elapsed(NaN),'—:—');
  assert.equal(present(hello,state()).elapsed,'00:35');
});
test('missing display name warns once per identifier',()=>{
  const warnings=[];const name=makeNames(hello,x=>warnings.push(x));
  assert.equal(name('reader'),'판독 드론');
  assert.equal(name('renamed'),'renamed');name('renamed');
  assert.equal(warnings.length,1);
});
test('LIMO goal uses the last reported nav command, never an inferred position',()=>{
  const value=state();
  assert.equal(navigationGoal(value,'rescuer'),'—');
  value.mission.cmd={rescuer:{kind:'nav',goal:[-1,1.5],t:30}};
  assert.equal(navigationGoal(value,'rescuer'),'−1.0, 1.5');
  value.mission.cmd.rescuer.kind='other';
  assert.equal(navigationGoal(value,'rescuer'),'—');
  value.mission.cmd.rescuer={kind:'nav',goal:[NaN,1.5]};
  assert.equal(navigationGoal(value,'rescuer'),'—');
});
