"""오프라인 시나리오 시뮬 하네스: 실제 modules/bt_constructor + bt_nodes 로 트리를 만들고,
가짜 ROS 노드가 명령을 받아 로봇 pose 를 순간이동/보간시키며 검출을 흉내낸다.
검증 목표: Phase 1→2→3→Reset 흐름, 재호출 없음, 게이팅 동작."""
import sys, os, math, asyncio, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'stubs'))
sys.path.insert(0, os.path.dirname(__file__))
os.environ['SDL_VIDEODRIVER'] = 'dummy'

from modules.utils import set_config
set_config('scenarios/coshow/configs/coshow_sim.yaml')
from modules.utils import config
config['coshow']['tolerances']['drone_hold'] = 0.2
config['coshow']['rescue_sec'] = 1.0
config['coshow']['search']['stagger_sec'] = 0.3

# ---- 가짜 ROS 노드/브릿지 ----
DEFERRED = []   # 실제 rclpy 처럼 콜백은 다음 spin(=다음 tick 사이)에 실행
class FakeFuture:
    def __init__(self): self._done=False; self._res=None; self._cbs=[]
    def done(self): return self._done
    def result(self): return self._res
    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self._done: DEFERRED.append(lambda: cb(self))
    def set(self, res):
        self._done=True; self._res=res
        for cb in self._cbs: DEFERRED.append(lambda cb=cb: cb(self))
def run_deferred():
    while DEFERRED: DEFERRED.pop(0)()

class FakeClock:
    def now(self):
        class T:
            def to_msg(self): return None
            nanoseconds = int(time.monotonic()*1e9)
        return T()

class FakeNode:
    def __init__(self):
        self.subs = {}; self.log = []; self.pending_nav = {}; self.srv_calls = []
    def create_subscription(self, t, topic, cb, q): self.subs.setdefault(topic, []).append(cb)
    def create_publisher(self, t, topic, q):
        n=self
        class P:
            def publish(self, m): n.log.append(('pub', topic, getattr(m,'data',None)))
        return P()
    def create_client(self, t, name):
        n=self
        class Cli:
            def wait_for_service(self, timeout_sec=0.0): return True
            def call_async(self, req):
                n.srv_calls.append((name, req)); n.log.append(('srv', name, req)); f=FakeFuture(); f.set(None); return f
        return Cli()
    def get_clock(self): return FakeClock()
    def make_goal_future(self, name, goal):
        f = FakeFuture()
        class GH:
            accepted=True
            def get_result_async(self2):
                rf=FakeFuture(); self.pending_nav[name]=(rf, goal); return rf
            def cancel_goal_async(self2): pass
        f.set(GH()); return f
    def deliver(self, topic, msg):
        for cb in self.subs.get(topic, []): cb(msg)

class FakeBridge:
    def __init__(self): self.node = FakeNode()

import modules.ros_bridge as rb
rb.ROSBridge.get = classmethod(lambda cls, *a, **k: cls._instance or setattr(cls,'_instance',FakeBridge()) or cls._instance)

from modules.agent import Agent
agent = Agent('')
agent.create_behavior_tree('scenarios/coshow/default_bt.xml')
node = agent.ros_bridge.node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from coshow_interfaces.msg import MarkerDetections
from action_msgs.msg import GoalStatus

# ---- 월드 상태 ----
world = {'cf_a':[-4.2,0.6,0.015],'cf_b':[-4.2,0.2,0.015],'cf_c':[-4.2,-0.2,0.015],'cf_d':[-4.2,-0.6,0.015],
         'limo_a':[-3.5,-2.0],'limo_b':[-3.5,-3.0]}
goal = {}
markers = {11: ('limo_a', None), 1: (None, (2.5, 2.5))}   # 11 은 limo_a 위, 1 은 바닥
SPEED = 3.0  # m/s (빠른 시뮬)

def step_world(dt):
    for r,g in list(goal.items()):
        p = world[r]
        for i in range(len(g)):
            d = g[i]-p[i]; p[i] += max(-SPEED*dt, min(SPEED*dt, d))
    # nav 결과: 도착 시 SUCCEEDED
    for name,(rf,gl) in list(node.pending_nav.items()):
        r = name.split('/')[1]
        gx,gy = gl.pose.pose.position.x, gl.pose.pose.position.y
        goal[r] = [gx,gy]
        if math.hypot(world[r][0]-gx, world[r][1]-gy) < 0.05:
            class Res: status=GoalStatus.STATUS_SUCCEEDED; result=None
            rf.set(Res()); del node.pending_nav[name]

def apply_srv():
    while node.srv_calls:
        name, req = node.srv_calls.pop(0)
        r, kind = name.split('/')[1], name.split('/')[2]
        p = world[r]
        if kind=='takeoff': goal[r] = [p[0],p[1],req.height]
        elif kind=='go_to': goal[r] = [req.goal.x, req.goal.y, req.goal.z]
        elif kind=='land': goal[r] = [p[0],p[1],0.015]

def publish():
    for d in ('cf_a','cf_b','cf_c','cf_d'):
        m = PoseStamped(); x,y,z = world[d]
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=z
        node.deliver(f'/{d}/pose', m)
        # 검출: 바로 아래 마커 (반경 0.3) 3프레임씩
        det = MarkerDetections(); det.markers=[]
        det.drone_pose.pose.position.x=x; det.drone_pose.pose.position.y=y; det.drone_pose.pose.position.z=z
        if z > 0.3:
            for mid,(carrier,pos) in markers.items():
                mx,my = world[carrier][:2] if carrier else pos
                if math.hypot(x-mx,y-my) < 0.45:
                    class Mk: id=mid
                    det.markers.append(Mk())
        node.deliver(f'/{d}/marker_detections', det)
    for l in ('limo_a','limo_b'):
        m = Odometry(); m.pose.pose.position.x, m.pose.pose.position.y = world[l]
        m.pose.pose.orientation.w=1.0; m.pose.pose.orientation.x=m.pose.pose.orientation.y=m.pose.pose.orientation.z=0.0
        node.deliver(l and f'/{l}/odom', m)

async def main():
    bb = agent.blackboard
    last = None; t0=time.monotonic()
    for i in range(6000):
        publish()
        st = await agent.run_tree()
        apply_srv(); step_world(0.05); run_deferred()
        key = (bb.get('mission_marker',{}).get('found'), bb.get('target_id'), bb.get('target_marker',{}).get('found'),
               bb.get('target_confirmed'), bb.get('finder'), bb.get('rescue_done_t',0)>0, bb.get('cycles_done',0))
        if key != last:
            print(f"t={time.monotonic()-t0:5.1f}s tick={i:4d} status={st.name:8s} mission={key[0]} target_id={key[1]} "
                  f"target_found={key[2]} confirmed={key[3]} finder={key[4]} rescued={key[5]} cycles={key[6]}")
            last = key
        if bb.get('cycles_done',0) >= 2: break
        await asyncio.sleep(0.005)
    # ---- 명령 로그 요약 ----
    srv = [l for l in node.log if l[0]=='srv']; nav=[l for l in node.log if l[0]=='nav']
    from collections import Counter
    print('\n서비스 호출 수:', Counter(l[1] for l in srv))
    print('Nav 목표:', [(n[1], round(n[2],2), round(n[3],2)) for n in nav])
    print('LED:', [l for l in node.log if l[0]=='pub'])
    print('search_progress:', bb.get('search_progress'))
    assert bb.get('cycles_done',0)>=2, 'cycle not completed'
    # 재호출 검사: 같은 (서비스, 목표) 연속 중복 없음
    dup = [srv[k][1] for k in range(1,len(srv)) if srv[k][1]==srv[k-1][1] and 'go_to' not in srv[k][1]]
    print('연속 중복 호출:', dup)

asyncio.run(main())
