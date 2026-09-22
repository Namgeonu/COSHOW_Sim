import * as THREE from '../vendor/three.module.js';
import {autoFrame,fieldBounds,roleIds,renderDue,placeLabels,TrailBuffer,PoseTrack,robotAppearance,SceneCues} from './twin-model.js';

// ROS (x,y,z) maps to three.js (x,z,-y). All scene locations come from hello/state.
export function createTwin(canvas,hello,{fx='',reducedMotion=false}={}) {
  const css=getComputedStyle(document.documentElement);
  const token=(key,fallback)=>css.getPropertyValue(key).trim()||fallback;
  const colors={ink:token('--ink','#F4F6FB'),cool:token('--light-cool','#5EE7FF'),
    warm:token('--light-warm','#FFB35C'),blue:token('--phase-search','#4DA3FF'),
    red:token('--phase-found','#FF5A5F'),green:token('--phase-rescue','#3DDC97'),off:token('--ink','#F4F6FB')};
  const renderer=new THREE.WebGLRenderer({canvas,alpha:true,antialias:fx!=='low',powerPreference:'low-power'});
  renderer.setPixelRatio(fx==='low'?.75:Math.min(globalThis.devicePixelRatio||1,1.5));
  renderer.setClearColor(0,0);renderer.shadowMap.enabled=false;
  const scene=new THREE.Scene(),camera=new THREE.OrthographicCamera(-1,1,1,-1,.01,150);
  const resources=new Set(),robotNodes=new Map(),lanes=new Map(),markers=new Map(),labels=[];
  const own=value=>(resources.add(value),value);
  const material=(color,extra={})=>own(new THREE.MeshStandardMaterial({color,roughness:.48,metalness:.22,...extra}));
  const basic=(color,extra={})=>own(new THREE.MeshBasicMaterial({color,...extra}));
  const mesh=(geometry,mat,parent=scene)=>{const item=new THREE.Mesh(own(geometry),mat);parent.add(item);return item;};
  const bounds=fieldBounds(hello.field),span=Math.max(bounds.x[1]-bounds.x[0],bounds.y[1]-bounds.y[0]);
  const ids=roleIds(hello);
  let disposed=false,frameId=0,frames=0,width=1,height=1,reservedBottom=0,lastTime=0,lastDraw=null,orbit=0;
  let state=null,view={mode:'idle'},received=0,foundAt=-Infinity,washAt=-Infinity;
  const cues=new SceneCues();
  const sceneHeight=Math.max(1,...ids.flatMap(id=>hello.lanes?.[id]||[]).map(p=>Number.isFinite(p[2])?p[2]:0))+.35;
  let framing=null;
  let labelBounds=[];
  const screenRight=new THREE.Vector3(),screenUp=new THREE.Vector3();

  scene.add(new THREE.HemisphereLight(colors.ink,'#080D20',2));
  const cool=new THREE.PointLight(colors.cool,span*span*.85,span*3,2);
  cool.position.set(bounds.x[1],3,-bounds.y[1]);scene.add(cool);
  const warm=new THREE.PointLight(colors.warm,span*span*.75,span*3,2);
  warm.position.set(bounds.x[0],2,-bounds.y[0]);scene.add(warm);

  // A single plinth joins the operational regions; inlays retain their real bounds.
  const plinth=mesh(new THREE.BoxGeometry(bounds.x[1]-bounds.x[0],.14,bounds.y[1]-bounds.y[0]),material('#101B30'));
  plinth.position.set((bounds.x[0]+bounds.x[1])/2,-.075,-(bounds.y[0]+bounds.y[1])/2);
  const rim=new THREE.LineSegments(own(new THREE.EdgesGeometry(plinth.geometry)),
    own(new THREE.LineBasicMaterial({color:colors.cool,transparent:true,opacity:.23})));
  rim.position.copy(plinth.position);scene.add(rim);
  const warmEdge=own(new THREE.BufferGeometry());warmEdge.setAttribute('position',new THREE.Float32BufferAttribute([
    bounds.x[0],-.005,-bounds.y[0],bounds.x[1],-.005,-bounds.y[0]],3));
  scene.add(new THREE.Line(warmEdge,own(new THREE.LineBasicMaterial({color:colors.warm,transparent:true,opacity:.22}))));
  function rectangle(region,color,grid=false) {
    if(!region?.x||!region?.y)return;
    const [x0,x1]=region.x,[y0,y1]=region.y;
    const floor=mesh(new THREE.PlaneGeometry(x1-x0,y1-y0),material(color));
    floor.rotation.x=-Math.PI/2;floor.position.set((x0+x1)/2,0,-(y0+y1)/2);
    if(grid) {
      const points=[];
      for(let x=x0;x<=x1+1e-7;x+=.5)points.push(x,.004,-y0,x,.004,-y1);
      for(let y=y0;y<=y1+1e-7;y+=.5)points.push(x0,.004,-y,x1,.004,-y);
      const geometry=own(new THREE.BufferGeometry());geometry.setAttribute('position',new THREE.Float32BufferAttribute(points,3));
      scene.add(new THREE.LineSegments(geometry,own(new THREE.LineBasicMaterial({color:colors.ink,transparent:true,opacity:.17}))));
      const wall=mesh(new THREE.BoxGeometry(x1-x0,.07,.018),material('#354866'));
      wall.position.set((x0+x1)/2,.035,-y1);
    }
  }
  rectangle(hello.field?.limo_area,'#111C2E');rectangle(hello.field?.arena,'#1A2A42',true);

  function billboard(label,scale=.2) {
    const surface=document.createElement('canvas');surface.width=1024;surface.height=128;
    const context=surface.getContext('2d'),texture=own(new THREE.CanvasTexture(surface));
    texture.colorSpace=THREE.SRGBColorSpace;
    const sprite=new THREE.Sprite(own(new THREE.SpriteMaterial({map:texture,transparent:true,depthWrite:false,depthTest:false})));
    const draw=()=>{
      context.clearRect(0,0,surface.width,surface.height);context.font='500 56px Pretendard, sans-serif';
      context.textAlign='center';context.textBaseline='middle';
      const textWidth=Math.min(980,context.measureText(label).width+40);
      sprite.userData.inkWidth=textWidth/1024;sprite.userData.inkHeight=84/128;
      context.fillStyle='rgba(14,21,48,.86)';context.beginPath();context.roundRect((1024-textWidth)/2,22,textWidth,84,18);context.fill();
      context.fillStyle=colors.ink;context.fillText(label,512,65,940);texture.needsUpdate=true;
    };
    draw();labels.push(draw);sprite.scale.set(scale*8,scale,1);return sprite;
  }
  // Font completion replaces pixels in existing textures; it never creates more textures.
  document.fonts?.ready.then(()=>{if(!disposed)for(const draw of labels)draw();});

  // 바닥 마커는 그리지 않는다. 실기 기준 마커 위치는 사전에 알 수 없고(수색 시나리오),
  // 시뮬 바닥에 미리 박힌 정사각형이 정답을 노출하므로 뺀다. 발견 후에는 foundRing 이
  // 실제 확정 위치(P_N)에만 잠깐 나타난다.
  if(Array.isArray(hello.observe_point)&&hello.observe_point.slice(0,2).every(Number.isFinite)) {
    const ring=mesh(new THREE.RingGeometry(.17,.19,48),basic(colors.cool,{transparent:true,opacity:.55,side:THREE.DoubleSide}));
    ring.rotation.x=-Math.PI/2;ring.position.set(hello.observe_point[0],.012,-hello.observe_point[1]);
  }
  for(const [id,points] of Object.entries(hello.lanes||{})) {
    if(!ids.includes(id))continue;
    const coords=points.filter(p=>Array.isArray(p)&&p.length>=3&&p.every(Number.isFinite)).flatMap(p=>[p[0],p[2],-p[1]]);
    if(coords.length<6)continue;
    const geometry=own(new THREE.BufferGeometry());geometry.setAttribute('position',new THREE.Float32BufferAttribute(coords,3));
    const line=new THREE.Line(geometry,own(new THREE.LineDashedMaterial({color:colors.ink,transparent:true,opacity:.2,dashSize:.09,gapSize:.08})));
    line.computeLineDistances();scene.add(line);lanes.set(id,line);
  }
  const foundRing=mesh(new THREE.RingGeometry(.15,.175,64),basic(colors.cool,{transparent:true,opacity:0,side:THREE.DoubleSide,depthWrite:false}));
  foundRing.rotation.x=-Math.PI/2;foundRing.visible=false;
  const arena=hello.field?.arena;
  const wash=mesh(new THREE.PlaneGeometry(arena?arena.x[1]-arena.x[0]:1,arena?arena.y[1]-arena.y[0]:1),
    basic(colors.green,{transparent:true,opacity:0,depthWrite:false,side:THREE.DoubleSide}));
  wash.rotation.x=-Math.PI/2;wash.visible=false;
  if(arena)wash.position.set((arena.x[0]+arena.x[1])/2,.009,-(arena.y[0]+arena.y[1])/2);

  function addRobot(id,kind,initial) {
    const group=new THREE.Group();scene.add(group);
    const bodyMat=material(kind==='drone'?'#9BC5D2':'#EDEFF6',{transparent:true});
    const ringMat=basic(colors.ink,{transparent:true,opacity:.4});
    if(kind==='drone') {
      for(const angle of [Math.PI/4,-Math.PI/4]) {const arm=mesh(new THREE.BoxGeometry(.32,.035,.046),bodyMat,group);arm.rotation.y=angle;}
      mesh(new THREE.BoxGeometry(.09,.055,.09),bodyMat,group);
      for(const x of [-1,1])for(const z of [-1,1]) {
        const motor=mesh(new THREE.CylinderGeometry(.034,.034,.025,12),bodyMat,group);motor.position.set(x*.108,0,z*.108);
      }
      const ring=mesh(new THREE.TorusGeometry(.165,.016,8,48),ringMat,group);ring.rotation.x=Math.PI/2;
    } else {
      const cabinMat=material('#DCE2F0',{transparent:true});
      const wheelMat=material('#1B2236',{transparent:true});
      const body=mesh(new THREE.BoxGeometry(.42,.13,.24),bodyMat,group);body.position.y=.02;
      const cabin=mesh(new THREE.BoxGeometry(.22,.11,.20),cabinMat,group);cabin.position.set(-.03,.14,0);
      const glass=mesh(new THREE.BoxGeometry(.06,.07,.17),material('#223A62',{transparent:true}),group);glass.position.set(.10,.14,0);
      for(const [wx,wz] of [[.14,.11],[.14,-.11],[-.14,.11],[-.14,-.11]]) {
        const w=mesh(new THREE.CylinderGeometry(.055,.055,.05,16),wheelMat,group);w.rotation.x=Math.PI/2;w.position.set(wx,-.03,wz);
      }
      for(const hz of [.06,-.06]) {
        const hl=mesh(new THREE.BoxGeometry(.01,.03,.04),material(colors.warm,{emissive:colors.warm,emissiveIntensity:1,transparent:true}),group);hl.position.set(.21,.02,hz);
      }
      // 미션마커를 싣고 전달하는 LIMO(첫 번째): 지붕에 노란 표식
      if((hello.limos||[])[0]===id) {
        const mk=mesh(new THREE.BoxGeometry(.16,.03,.16),material(colors.warm,{emissive:colors.warm,emissiveIntensity:.6,transparent:true}),group);mk.position.set(-.03,.22,0);
      }
    }
    const halo=mesh(new THREE.SphereGeometry(kind==='drone'?.28:.34,20,14),basic(colors.cool,{transparent:true,opacity:0,depthWrite:false}),group);
    halo.position.y=kind==='drone'?0:.08;
    const label=billboard(hello.display_names?.[id]||id,.32);label.renderOrder=3;scene.add(label);
    const leaderGeometry=own(new THREE.BufferGeometry());leaderGeometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(6),3));
    const leader=new THREE.Line(leaderGeometry,own(new THREE.LineBasicMaterial({color:colors.ink,transparent:true,opacity:.2,depthWrite:false,depthTest:false})));
    leader.frustumCulled=false;leader.visible=false;leader.renderOrder=2;scene.add(leader);
    const shadow=mesh(new THREE.CircleGeometry(kind==='drone'?.04:.09,20),basic(colors.cool,{transparent:true,opacity:.38,depthWrite:false}));
    shadow.rotation.x=-Math.PI/2;
    // 비행 궤적(trail)은 그리지 않는다 — 요청에 따라 제거. 고도 표시선(stem)만 유지.
    let stem=null,trail=null,trailLine=null;
    if(kind==='drone') {
      const stemGeometry=own(new THREE.BufferGeometry());stemGeometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(6),3));
      stem=new THREE.Line(stemGeometry,own(new THREE.LineBasicMaterial({color:colors.cool,transparent:true,opacity:.32})));
      stem.frustumCulled=false;scene.add(stem);
    }
    const materials=[];group.traverse(item=>{if(item.material&&!materials.includes(item.material))materials.push(item.material);});
    const entry={kind,group,bodyMat,ringMat,label,leader,halo,labelOffset:kind==='drone'?.3:.35,
      anchor:new THREE.Vector3(),projected:new THREE.Vector3(),shadow,stem,trail,trailLine,materials,
      track:new PoseTrack(initial),row:{kind,role:null},lastTrail:-Infinity};
    robotNodes.set(id,entry);return entry;
  }

  function syncRobots() {
    const rows=state?.robots||{};
    for(const id of ids) {
      const kind=rows[id]?.kind||((hello.drones||[]).includes(id)?'drone':'limo');
      const base=hello.bases?.[id];
      const fallback={x:base?.[0]??0,y:base?.[1]??0,z:base?.[2]??0};
      const node=robotNodes.get(id)||addRobot(id,kind,{...fallback,yaw:0});
      node.row=rows[id]||{kind,role:id,pose:null,pose_age:null};
      node.group.visible=node.shadow.visible=true;
      const appearance=robotAppearance(hello,id,node.row,state,view,0);
      if(appearance.fresh) {
        node.track.update(node.row.pose,received);
      }
    }
    for(const [id,node] of robotNodes)if(!ids.includes(id)) {
      node.group.visible=node.shadow.visible=node.label.visible=node.leader.visible=false;if(node.stem)node.stem.visible=false;if(node.trailLine)node.trailLine.visible=false;
    }
  }

  function layoutLabels() {
    camera.updateMatrixWorld();
    screenRight.setFromMatrixColumn(camera.matrixWorld,0);screenUp.setFromMatrixColumn(camera.matrixWorld,1);
    const pixelsPerMetre=height/(camera.top-camera.bottom),items=[];
    for(const [id,node] of robotNodes)if(node.group.visible) {
      node.anchor.copy(node.group.position);node.anchor.y+=node.labelOffset;
      node.projected.copy(node.anchor).project(camera);
      const x=(node.projected.x+1)*width/2,y=(1-node.projected.y)*height/2;
      items.push({id,x,y,width:node.label.scale.x*node.label.userData.inkWidth*pixelsPerMetre,
        height:node.label.scale.y*node.label.userData.inkHeight*pixelsPerMetre});
    }
    labelBounds=placeLabels(items,{width,height:Math.max(height*.4,height-reservedBottom)});
    for(let index=0;index<labelBounds.length;index++) {
      const box=labelBounds[index],original=items[index],node=robotNodes.get(box.id);
      node.label.visible=box.visible;
      node.label.position.copy(node.anchor).addScaledVector(screenRight,(box.x-original.x)/pixelsPerMetre)
        .addScaledVector(screenUp,(original.y-box.y)/pixelsPerMetre);
      node.leader.visible=box.visible&&Math.hypot(box.x-original.x,box.y-original.y)>8;
      if(node.leader.visible) {
        const attribute=node.leader.geometry.attributes.position;
        attribute.array.set([...node.group.position.toArray(),...node.label.position.toArray()]);attribute.needsUpdate=true;
      }
    }
  }

  function frameCamera() {
    const visibleHeight=Math.max(height*.4,height-reservedBottom);
    const frame=autoFrame(hello.field,width/visibleHeight,{height:sceneHeight,azimuth:Math.PI/8+orbit});
    framing={fieldBounds:bounds,visibleHeight,azimuth:Math.PI/8+orbit,elevation:.48,
      projectedWidth:frame.projectedWidth,projectedHeight:frame.projectedHeight,
      widthFraction:frame.projectedWidth/(frame.halfWidth*2),heightFraction:frame.projectedHeight/(frame.halfHeight*2)};
    camera.left=-frame.halfWidth;camera.right=frame.halfWidth;camera.top=frame.halfHeight;
    camera.bottom=frame.halfHeight-2*frame.halfHeight*height/visibleHeight;
    camera.position.set(...frame.target).addScaledVector(new THREE.Vector3(...frame.direction),Math.max(20,span*3));
    camera.lookAt(...frame.target);camera.updateProjectionMatrix();
  }
  function resize(w,h,bottom=0) {
    if(disposed||!Number.isFinite(w)||!Number.isFinite(h)||w<=0||h<=0)return;
    if(w===width&&h===height&&bottom===reservedBottom)return;
    width=w;height=h;reservedBottom=Math.max(0,bottom);renderer.setSize(w,h,false);frameCamera();
  }
  function update(next,nextView) {
    if(disposed)return;
    state=next;view=nextView||{mode:'idle'};received=performance.now()/1000;
    syncRobots();
    const event=cues.update(state,view);
    if(view.mode==='idle')for(const node of robotNodes.values())node.trail?.clear();
    if(event.found) {
      const point=state.mission.P_N;foundAt=received;foundRing.position.set(point.x,.025,-point.y);
      foundRing.material.color.set(colors[state.mission.led?.[state.mission.finder]]||colors.cool);
    }
    if(event.wash)washAt=received;
    if(!event.visible) {foundAt=washAt=-Infinity;foundRing.visible=wash.visible=false;}
  }
  function render(milliseconds) {
    if(disposed)return;
    frameId=requestAnimationFrame(render);
    if(!renderDue(milliseconds,lastDraw))return;
    lastDraw=milliseconds;
    const now=milliseconds/1000,delta=lastTime?Math.min(.1,now-lastTime):0;lastTime=now;
    if(!reducedMotion) {orbit+=delta*.05;frameCamera();}   // 전체 판 저속 회전 (모든 국면)
    const age=Math.max(0,now-received),active=view.mode==='active' && (state?.run?.state==='DONE'||(state?.mission?.age??Infinity)+age<=3);
    for(const [id,node] of robotNodes) {
      if(!node.group.visible)continue;
      const appearance=robotAppearance(hello,id,node.row,state,view,age),position=node.track.sample(now);
      node.group.position.set(position.x,Math.max(0,position.z)+(node.kind==='drone'?.035:.105),-position.y);
      node.group.rotation.y=position.yaw;
      for(const mat of node.materials)mat.opacity=appearance.opacity;
      node.label.material.opacity=appearance.opacity;
      node.leader.material.opacity=appearance.opacity*.2;
      node.ringMat.color.set(colors[appearance.signal]);node.ringMat.opacity=appearance.opacity*(appearance.signal==='off'?.4:1);
      // 후레시 명멸 헤일로: 탐색(파랑,은은)·발견(빨강,강하게)·구출(초록)·복귀(흰색)
      let flash=appearance.signal;
      if(active&&['return','done'].includes(view.phase)&&node.kind==='drone'&&position.z>.05)flash='white';
      node.halo.material.color.set(flash==='white'?colors.ink:colors[flash]);
      let amp=0;
      if(flash==='red')amp=(now*3%1)<.5?.6:.12;
      else if(flash==='green')amp=(Math.sin(now*Math.PI*2*1.8)+1)/2*.4+.12;
      else if(flash==='blue')amp=(Math.sin(now*Math.PI*2*0.9)+1)/2*.22+.05;
      else if(flash==='white')amp=(Math.sin(now*Math.PI*2*1.3)+1)/2*.32+.08;
      node.halo.material.opacity=reducedMotion?(amp>0?.22:0):amp*appearance.opacity;
      node.halo.scale.setScalar(1+amp*(flash==='red'?.4:.15));
      node.shadow.position.set(position.x,.014,-position.y);node.shadow.material.opacity=appearance.opacity*.38;
      if(node.stem) {
        node.stem.visible=appearance.stem;
        const attribute=node.stem.geometry.attributes.position;
        attribute.array.set([position.x,.014,-position.y,position.x,Math.max(0,position.z),-position.y]);attribute.needsUpdate=true;
      }
      const lane=lanes.get(id);
      if(lane) {
        const searching=active&&view.phase==='search'&&appearance.fresh&&position.z>.05;
        lane.material.color.set(searching?colors.blue:colors.ink);lane.material.opacity=searching?.7:.18;
      }
    }
    const ringProgress=(now-foundAt)/1.8;
    foundRing.visible=active&&ringProgress>=0&&ringProgress<1;
    if(foundRing.visible) {foundRing.scale.setScalar(1+ringProgress*5);foundRing.material.opacity=(1-ringProgress)*.85;}
    const washProgress=(now-washAt)/2.4;wash.visible=active&&washProgress>=0&&washProgress<1;
    if(wash.visible)wash.material.opacity=Math.sin(washProgress*Math.PI)*.2;
    layoutLabels();
    renderer.render(scene,camera);frames++;
  }
  const contextLost=event=>{event.preventDefault();if(!disposed)location.reload();};
  canvas.addEventListener('webglcontextlost',contextLost);
  syncRobots();frameCamera();resize(canvas.clientWidth||1,canvas.clientHeight||1);frameId=requestAnimationFrame(render);
  function dispose() {
    if(disposed)return;disposed=true;cancelAnimationFrame(frameId);canvas.removeEventListener('webglcontextlost',contextLost);
    for(const resource of resources)resource.dispose();resources.clear();labels.length=0;robotNodes.clear();lanes.clear();markers.clear();scene.clear();
    renderer.renderLists.dispose();renderer.dispose();renderer.forceContextLoss();
  }
  function stats() {
    return {geometries:renderer.info.memory.geometries,textures:renderer.info.memory.textures,programs:renderer.info.programs?.length||0,
      drawCalls:renderer.info.render.calls,triangles:renderer.info.render.triangles,robots:robotNodes.size,
      droneTrails:[...robotNodes.values()].filter(node=>node.trail).length,trailCapacity:128,frames,
      pixelRatio:renderer.getPixelRatio(),width,height,reservedBottom,orbit,disposed,
      rolesOnly:true,roleIds:[...ids],framing,labelBounds,spares:[],maxRenderFps:60};
  }
  return {update,resize,dispose,stats};
}
