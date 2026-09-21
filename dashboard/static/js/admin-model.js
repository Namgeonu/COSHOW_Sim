const acceptance={IDLE:['preflight','reset'],CHECKING:['estop','reset'],READY:['start','estop','reset'],
  RUNNING:['estop','reset'],LANDING:[],DONE:['estop','reset'],ABORTED:['reset']};
export function checklistRows(state) {
  const rows=[],spareWarnings=[];
  for(const row of state.checklist||[]) {
    const robot=state.robots?.[row.group];
    if(robot&&robot.role==null) {if(row.status==='warning')spareWarnings.push(row);}
    else rows.push(row);
  }
  return {rows,spareWarnings};
}
export function buttons(state,connected,pending) {
  return Object.fromEntries(['preflight','start','estop','reset'].map(cmd=>[cmd,Boolean(connected &&
    acceptance[state?.run?.state]?.includes(cmd) && (!pending || cmd==='estop'&&pending.elapsed>=1000) &&
    (cmd!=='start'||!(state?.checklist||[]).some(row=>row.blocking&&!row.ok)))]));
}
export function commandReply(events,pending) {
  if(!pending)return null;
  return [...events].reverse().find(event=>!pending.seen.has(`${event.t}|${event.text}`)&&
    event.text.startsWith(`cmd:${pending.cmd} `))||null;
}
export function landingText(state) {
  const recent=[...(state.events||[])].reverse();
  const boundary=recent.findIndex(event=>event.text==='착륙 시퀀스 시작');
  const sent=boundary<0?null:recent.slice(0,boundary).find(event=>/land 전송 \d+\/\d+/.test(event.text));
  return `착륙 중 ${(state.run?.elapsed_s||0).toFixed(1)}초 · ${sent?.text.match(/land 전송 \d+\/\d+/)?.[0]||'land 전송 대기'}`;
}
