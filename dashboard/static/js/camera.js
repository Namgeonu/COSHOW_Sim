// At most one ImageBitmap and one pending decode per tile; no frame queue.
export function cameraDecoder(canvas,{decode=createImageBitmap,onSize=()=>{}}={}) {
  const context=canvas.getContext('2d',{alpha:false});
  let busy=false,disposed=false,received=false,dropped=0;
  return {
    get received(){return received;},get dropped(){return dropped;},
    dispose(){disposed=true;},
    async draw(bytes) {
      if(disposed)return false;
      if(busy){dropped++;return false;}
      busy=true;let bitmap;
      try {
        bitmap=await decode(new Blob([bytes],{type:'image/jpeg'}));
        if(disposed)return false;
        if(canvas.width!==bitmap.width || canvas.height!==bitmap.height){
          canvas.width=bitmap.width;canvas.height=bitmap.height;onSize(bitmap.width,bitmap.height);
        }
        context.drawImage(bitmap,0,0);received=true;return true;
      } catch {return false;}
      finally {bitmap?.close();busy=false;}
    },
  };
}

export function cameraMessage(received,camera,threshold,connected) {
  if(!received)return '카메라 대기';
  return !connected || camera?.stream_ok===false || !Number.isFinite(camera?.frame_age) || camera.frame_age>threshold ? '수신 끊김' : '';
}
