// Pure geometry and display timing. Coordinates arrive only through the protocol.
const valid = p => p && ['x','y','z'].every(key => Number.isFinite(p[key]));
const pose = p => ({x:p.x,y:p.y,z:p.z,yaw:Number.isFinite(p.yaw)?p.yaw:0});
export function roleIds(hello={}) {return [...new Set([...(hello.drones||[]),...(hello.limos||[])])];}
export function renderDue(now,last) {return last==null||now-last>=1000/60-1e-7;}
export function placeLabels(items,{width,height,margin=8,gap=4}) {
  const placed=[],clamp=(n,lo,hi)=>Math.max(lo,Math.min(hi,n));
  for(const item of items) {
    const w=item.width,h=item.height;
    if(w>width-2*margin||h>height-2*margin) {placed.push({...item,visible:false});continue;}
    const x=clamp(item.x,margin+w/2,width-margin-w/2);
    const y=clamp(item.y,margin+h/2,height-margin-h/2);
    const clear=candidate=>placed.every(other=>!other.visible||
      Math.abs(candidate.x-other.x)>=(w+other.width)/2+gap||Math.abs(candidate.y-other.y)>=(h+other.height)/2+gap);
    const xs=[x];
    for(let step=1;step*w<width;step++)for(const sign of [-1,1]) {
      const at=x+sign*step*(w+gap);
      if(at>=margin+w/2&&at<=width-margin-w/2)xs.push(at);
    }
    let chosen=null;
    for(const at of xs) {
      const ys=[y];
      for(let up=y-h-gap;up>=margin+h/2;up-=h+gap)ys.push(up);
      for(let down=y+h+gap;down<=height-margin-h/2;down+=h+gap)ys.push(down);
      for(const top of ys)if(clear({x:at,y:top})) {chosen={...item,x:at,y:top,visible:true};break;}
      if(chosen)break;
    }
    placed.push(chosen||{...item,x,y,visible:false});
  }
  return placed;
}
export function fieldBounds(field = {}) {
  const regions = ['arena','limo_area'].map(key=>field[key]).filter(region=>
    region && ['x','y'].every(axis=>Array.isArray(region[axis]) && region[axis].length===2 && region[axis].every(Number.isFinite)));
  if (!regions.length) return {x:[-.5,.5],y:[-.5,.5]};
  return Object.fromEntries(['x','y'].map(axis=>[axis,[Math.min(...regions.flatMap(r=>r[axis])),Math.max(...regions.flatMap(r=>r[axis]))]]));
}

export function autoFrame(field, aspect, {height=2,azimuth=Math.PI/8,elevation=.48,floor=-.14}={}) {
  const bounds=fieldBounds(field),c=Math.cos(azimuth),s=Math.sin(azimuth),ce=Math.cos(elevation),se=Math.sin(elevation);
  const target=[(bounds.x[0]+bounds.x[1])/2,(height+floor)/2,-(bounds.y[0]+bounds.y[1])/2];
  const right=[c,0,-s],up=[-s*se,ce,-c*se],direction=[s*ce,se,c*ce];
  let projectedWidth=0,projectedHeight=0;
  for(const x of bounds.x) for(const y of bounds.y) for(const z of [floor,height]) {
    const p=[x-target[0],z-target[1],-y-target[2]];
    projectedWidth=Math.max(projectedWidth,Math.abs(p.reduce((sum,n,i)=>sum+n*right[i],0)));
    projectedHeight=Math.max(projectedHeight,Math.abs(p.reduce((sum,n,i)=>sum+n*up[i],0)));
  }
  const safeAspect=Number.isFinite(aspect)&&aspect>0?aspect:1;
  const halfHeight=Math.max(projectedHeight,projectedWidth/safeAspect,.1)*1.035;
  return {target,right,up,direction,halfHeight,halfWidth:halfHeight*safeAspect,
    projectedWidth:projectedWidth*2,projectedHeight:projectedHeight*2};
}

export class TrailBuffer {
  constructor(capacity=128,seconds=10) {
    this.capacity=capacity;this.seconds=seconds;this.head=0;this.length=0;
    this.positions=new Float32Array(capacity*3);this.times=new Float64Array(capacity);
  }
  push(t,p) {
    if(!valid(p)||!Number.isFinite(t)) return;
    const at=this.head*3;
    this.positions[at]=p.x;this.positions[at+1]=p.z;this.positions[at+2]=-p.y;
    this.times[this.head]=t;this.head=(this.head+1)%this.capacity;
    this.length=Math.min(this.length+1,this.capacity);
  }
  // Write adjacent pairs into preallocated GPU attributes; never join across wrap.
  writeSegments(positions,times,now) {
    let count=0,previous=-1;
    for(let i=0;i<this.length;i++) {
      const index=(this.head-this.length+i+this.capacity)%this.capacity;
      if(now-this.times[index]>this.seconds+1e-7) {previous=-1;continue;}
      if(previous>=0) for(const at of [previous,index]) {
        positions.set(this.positions.subarray(at*3,at*3+3),count*3);times[count++]=this.times[at];
      }
      previous=index;
    }
    return count;
  }
  clear() {this.head=0;this.length=0;}
}

export class PoseTrack {
  constructor(initial={x:0,y:0,z:0,yaw:0}) {this.from=pose(initial);this.to=pose(initial);this.at=-Infinity;}
  update(value,now) {
    if(!valid(value)) return false;
    this.from=this.sample(now);this.to=pose(value);this.at=now;return true;
  }
  sample(now) {
    const alpha=Math.max(0,Math.min(1,(now-this.at)/.1));
    const yawDelta=Math.atan2(Math.sin(this.to.yaw-this.from.yaw),Math.cos(this.to.yaw-this.from.yaw));
    return {x:this.from.x+(this.to.x-this.from.x)*alpha,y:this.from.y+(this.to.y-this.from.y)*alpha,
      z:this.from.z+(this.to.z-this.from.z)*alpha,yaw:this.from.yaw+yawDelta*alpha};
  }
}

export function robotAppearance(hello,id,row,state,view,elapsed=0) {
  const threshold=hello.freshness_s?.[row.kind==='limo'?'odom':'pose']??1;
  const fresh=Boolean(valid(row.pose)&&Number.isFinite(row.pose_age)&&row.pose_age+elapsed<=threshold);
  const visible=fresh&&view.mode!=='offline';
  const allowed=view.mode==='active'&&!['LANDING','ABORTED'].includes(state?.run?.state);
  const missionFresh=state?.run?.state==='DONE'||(Number.isFinite(state?.mission?.age)&&state.mission.age+elapsed<=3);
  const signal=allowed&&fresh&&missionFresh?state?.mission?.led?.[id]:'off';
  return {fresh,opacity:visible?(row.role==null?.5:1):.28,stem:visible&&row.kind==='drone',
    signal:['blue','red','green'].includes(signal)?signal:'off'};
}

export class SceneCues {
  constructor() {this.reset();}
  reset() {this.found=false;this.washed=false;this.run=null;}
  update(state,view) {
    if(view.mode==='idle'&&!state?.run?.external_bt) this.reset();
    const since=state?.run?.since;
    if(since!=null&&this.run!=null&&since!==this.run && !['return','done'].includes(view.phase)) this.reset();
    if(since!=null) this.run=since;
    const visible=view.mode==='active';
    const point=state?.mission?.P_N;
    const found=Boolean(visible&&!this.found&&Number.isFinite(point?.x)&&Number.isFinite(point?.y));
    const wash=Boolean(visible&&!this.washed&&['return','done'].includes(view.phase));
    if(found)this.found=true;if(wash)this.washed=true;
    return {found,wash,visible};
  }
}
