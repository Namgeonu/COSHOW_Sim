import test from 'node:test';
import assert from 'node:assert/strict';
import {cameraDecoder,cameraMessage} from '../static/js/camera.js';

test('binary JPEG drops while busy, resizes to decoded dimensions, draws and closes',async()=>{
  let resolve,closed=0,draws=0,sizes=0;
  const canvas={width:324,height:244,getContext:()=>({drawImage:()=>draws++})};
  const decoder=cameraDecoder(canvas,{decode:()=>new Promise(r=>resolve=r),onSize:()=>sizes++});
  const first=decoder.draw(new Uint8Array([255,216,255,217]));
  assert.equal(await decoder.draw(new Uint8Array([255,216])),false);
  resolve({width:320,height:220,close:()=>closed++});
  assert.equal(await first,true);
  assert.equal(draws,1);assert.equal(closed,1);assert.equal(sizes,1);
  assert.deepEqual([canvas.width,canvas.height],[320,220]);
  assert.equal(decoder.received,true);assert.equal(decoder.dropped,1);
});
test('disposed decoder closes late bitmap without touching detached canvas',async()=>{
  let resolve,closed=0;
  const decoder=cameraDecoder({getContext:()=>({drawImage:()=>assert.fail('detached draw')})},
    {decode:()=>new Promise(r=>resolve=r)});
  const pending=decoder.draw(new Uint8Array([1]));decoder.dispose();
  resolve({width:10,height:10,close:()=>closed++});
  assert.equal(await pending,false);assert.equal(closed,1);
});
test('failed decoding frees its slot and overlay distinguishes never received from stale',async()=>{
  const decoder=cameraDecoder({getContext:()=>({})},{decode:async()=>{throw Error('bad JPEG');}});
  assert.equal(await decoder.draw(new Uint8Array([1])),false);
  assert.equal(await decoder.draw(new Uint8Array([1])),false);
  assert.equal(decoder.dropped,0);
  assert.equal(cameraMessage(false,{},2,true),'카메라 대기');
  assert.equal(cameraMessage(true,{stream_ok:false,frame_age:0},2,true),'수신 끊김');
  assert.equal(cameraMessage(true,{stream_ok:true,frame_age:3},2,true),'수신 끊김');
  assert.equal(cameraMessage(true,{stream_ok:true,frame_age:0},2,false),'수신 끊김');
  assert.equal(cameraMessage(true,{stream_ok:true,frame_age:0},2,true),'');
});
