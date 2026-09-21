// One connection with bounded reconnect timers; JPEGs are never queued here.
export function connect({onHello, onState, onConnection, onFrame=()=>{},role='visitor'}) {
  let socket = null;
  let retry = null;
  let expiry = null;
  let stopped = false;
  let lastState = null;
  let socketOpenedAt = null;
  let socketLastState = null;
  let hasHello = false;
  let connected = false;
  let attempt = 0;
  function notify(value) {
    if (connected === value) return;
    connected = value;
    onConnection(value);
  }
  function adminDeadline(current) {
    if (role !== 'admin' || stopped || socket !== current) return;
    clearTimeout(expiry);
    const receipt = socketLastState ?? socketOpenedAt;
    if (receipt === null) return;
    const remaining = 1500 - (performance.now() - receipt);
    if (remaining > 0) {
      expiry = setTimeout(() => adminDeadline(current), remaining);
      return;
    }
    notify(false);
    if (current.readyState === WebSocket.OPEN) current.close();
  }
  function open() {
    if (stopped) return;
    clearTimeout(expiry);
    hasHello = false;
    socketOpenedAt = null;
    socketLastState = null;
    const address = new URL('/ws?role='+encodeURIComponent(role), location.href);
    address.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const current = new WebSocket(address);
    socket = current;
    current.binaryType = 'arraybuffer';
    current.onopen = () => {
      if (socket === current) {
        socketOpenedAt = performance.now();
        adminDeadline(current);
      }
    };
    current.onmessage = event => {
      if (socket !== current) return;
      if (event.data instanceof ArrayBuffer) {
        if(hasHello && event.data.byteLength>1) {
          const bytes=new Uint8Array(event.data);onFrame(bytes[0],bytes.subarray(1));
        }
        return;
      }
      if(typeof event.data!=='string') return;
      try {
        const value = JSON.parse(event.data);
        if (value.type === 'hello') {
          hasHello = true;
          onHello(value);
        } else if (value.type === 'state' && hasHello) {
          const receivedAt = performance.now();
          onState(value);
          lastState = receivedAt;
          socketLastState = lastState;
          attempt = 0;
          notify(true);
          adminDeadline(current);
        }
      } catch (error) {
        console.warn('대시보드 수신 처리 오류', error);
      }
    };
    current.onerror = () => current.close();
    current.onclose = () => {
      if (socket !== current || stopped) return;
      clearTimeout(expiry);
      if(role==='admin')notify(false);
      // Keep the visitor's last scene for the specified two-second grace.
      clearTimeout(retry);
      retry = setTimeout(open, Math.min(5000, 500 * 2 ** attempt++));
    };
  }
  onConnection(false);
  open();
  const watchdog = role === 'admin' ? null : setInterval(() => {
    const now = performance.now();
    const expired=age=>age>2000;
    if (lastState !== null && expired(now - lastState)) {
      notify(false);
    }
    // Each new connection gets its own grace period, including its first state.
    // The previous scene's age must not close a freshly reconnected socket.
    const deadlineStart = socketLastState ?? socketOpenedAt;
    if (deadlineStart !== null && expired(now - deadlineStart)
        && socket?.readyState === WebSocket.OPEN) socket.close();
  }, 250);
  const stop = () => {
    stopped = true;
    clearInterval(watchdog);
    clearTimeout(retry);
    clearTimeout(expiry);
    socket?.close();
  };
  stop.send=cmd=>{
    if(stopped||role!=='admin'||!connected||socket?.readyState!==WebSocket.OPEN)return false;
    // Timer callbacks can be delayed in a background tab; never send from stale state.
    if(socketLastState===null||performance.now()-socketLastState>=1500) {
      adminDeadline(socket);return false;
    }
    socket.send(JSON.stringify({cmd}));return true;
  };
  return stop;
}
