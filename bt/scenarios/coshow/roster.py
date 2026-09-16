"""투입 기체 선택 — 역할(슬롯)과 물리 기체의 분리.

config['coshow']['slots'] 가 역할별 자리(base)·고도·탐색 구역을 정의한다.
어떤 기체가 어느 슬롯에 들어갈지는 실행마다 정한다 (프롬프트 4칸 또는 CLI 인자).

    slots:                                   실행 시 꽂는 기체
      mission   base (-1.0, 0.0)  ...   ←  cf232
      search_a  base (-1.5, 1.5)  zone  ←  cf230
      search_b  base (-1.5, 0.0)  zone  ←  cf235
      search_c  base (-1.5,-1.5)  zone  ←  cf237

apply_roster() 는 main.py 가 set_config() 직후, BTRunner 를 import 하기 **전에** 부른다.
bt_nodes.py 는 import 시점에 C['drones'] / C['observe_drone'] / C['searchers'] /
C['search']['zones'] 를 읽으므로, 여기서 slots 를 그 네 키로 풀어 써두면 트리 코드는
한 줄도 안 고치고 새 명단으로 돈다. 선택되지 않은 기체는 drones 에 남기지 않는다
(UpdateBlackboard 가 DRONES 전원의 pose 를 요구해, 안 뜬 기체가 남으면 BT 가 시작을 못 한다).

시뮬 제약: Webots 스폰 위치는 월드 파일에 박혀 있어 기체를 바꿔 꽂아도 출발 자리는 그대로다.
slots.base 는 복귀 목적지로만 작동한다. 실기체는 사람이 슬롯 자리에 놓고 시작하므로 문제없다.

사용:
  python3 main.py --config ...                      # roster.enabled 면 프롬프트, 아니면 default 명단
  python3 main.py --observe cf235 --searchers cf230,cf232,cf237
  python3 main.py --roster / --no-roster            # 프롬프트 강제 / 생략
"""
import re
import sys

_NAME = re.compile(r'^cf\d+$')
_OBS_SLOT = 'mission'


def _norm(s):
    """'232' 도 'cf232' 로 받아준다."""
    s = s.strip()
    return f'cf{s}' if s.isdigit() else s


def _label(slot):
    if slot == _OBS_SLOT:
        return 'Mission Drone'
    return 'Search Drone_' + slot.split('_', 1)[1] if '_' in slot else slot


def _validate(mapping, available):
    names = list(mapping.values())
    bad = [x for x in names if not _NAME.match(x)]
    if bad:
        return f'이름 형식 오류: {bad} (cf 뒤에 숫자, 예: cf235 또는 235)'
    if available:
        unknown = [x for x in names if x not in available]
        if unknown:
            return f'보유 목록에 없음: {unknown}'
    if len(set(names)) != len(names):
        dup = sorted({x for x in names if names.count(x) > 1})
        return f'같은 기체를 두 슬롯에 넣었다: {dup}'
    return None


def _describe(c, mapping):
    slots = c['slots']
    w = max(len(_label(s)) for s in slots)
    lines = []
    for s, d in mapping.items():
        sl = slots[s]
        extra = f"  base {sl.get('base')}"
        if 'zone' in sl:
            extra += f"  zone x{sl['zone']['x']} y{sl['zone']['y']}"
        if 'altitudes' in sl:
            extra += f"  alt {sl['altitudes']}"
        lines.append(f'{_label(s):<{w}} : {d:<7}{extra}')
    return lines


def _prompt(c, available, defaults):
    slots = c['slots']
    w = max(len(_label(s)) for s in slots)
    print('\n═══ 투입 기체 선택 ═══', flush=True)
    if available:
        print('보유 기체:', ' '.join(available), flush=True)
    print('(숫자만 입력해도 된다: 232 → cf232. 빈 칸은 기본 배정)', flush=True)
    while True:
        mapping = {}
        for s in slots:
            raw = input(f'{_label(s):<{w}} : ')
            mapping[s] = _norm(raw) if raw.strip() else defaults[s]
        err = _validate(mapping, available)
        if err:
            print(f'  ✗ {err}. 다시 입력.\n', flush=True)
            continue
        print(flush=True)
        for line in _describe(c, mapping):
            print('  ' + line, flush=True)
        ans = input('확인? [Y/n]: ').strip().lower()
        if ans in ('', 'y', 'yes'):
            return mapping
        print(flush=True)


def _materialize(c, mapping):
    """slots + mapping → bt_nodes 가 읽는 legacy 키 (drones / observe_drone / searchers / search.zones)."""
    slots = c['slots']
    search_slots = [s for s in slots if s != _OBS_SLOT]        # yaml 순서 = 출격 순서 = 구역 순서
    drones = {}
    for s in [_OBS_SLOT] + search_slots:
        drones[mapping[s]] = {k: v for k, v in slots[s].items() if k not in ('default', 'zone')}
    c['drones'] = drones
    c['observe_drone'] = mapping[_OBS_SLOT]
    c['searchers'] = [mapping[s] for s in search_slots]
    c.setdefault('search', {})['zones'] = {mapping[s]: slots[s]['zone'] for s in search_slots}
    c['slot_of'] = {d: s for s, d in mapping.items()}          # 로그·표시용 역방향 표


def apply_roster(config, observe=None, searchers=None, interactive=True, force_prompt=False):
    """config['coshow'] 를 제자리에서 고친다. slots 가 없는 (legacy) config 는 건드리지 않는다.

    우선순위: CLI 인자(observe/searchers) > 프롬프트(roster.enabled 또는 force_prompt) > slots.default.
    stdin 이 터미널이 아니면(자동 실행) 프롬프트를 띄우지 않고 default 를 쓴다.
    """
    c = config.get('coshow')
    if not c or 'slots' not in c:
        return
    slots = c['slots']
    if _OBS_SLOT not in slots:
        raise SystemExit(f"[ROSTER] slots 에 '{_OBS_SLOT}' 슬롯이 없다")
    r = c.get('roster') or {}
    available = [_norm(x) for x in (r.get('available') or [])]
    defaults = {s: slots[s]['default'] for s in slots}
    search_slots = [s for s in slots if s != _OBS_SLOT]

    if observe is not None or searchers is not None:
        mapping = dict(defaults)
        if observe is not None:
            mapping[_OBS_SLOT] = _norm(observe)
        if searchers is not None:
            searchers = [_norm(x) for x in searchers]
            if len(searchers) != len(search_slots):
                raise SystemExit(f'[ROSTER] 탐색 담당은 {len(search_slots)}대여야 한다 (입력 {len(searchers)}대)')
            mapping.update(zip(search_slots, searchers))
        err = _validate(mapping, available)
        if err:
            raise SystemExit(f'[ROSTER] {err}')
        src = 'CLI'
    else:
        want_prompt = force_prompt or bool(r.get('enabled', False))
        if want_prompt and interactive and sys.stdin.isatty():
            mapping = _prompt(c, available, defaults)
            src = '프롬프트'
        else:
            if want_prompt:
                print('[ROSTER] 프롬프트 생략 (비대화형). slots.default 사용', flush=True)
            mapping = dict(defaults)
            src = '기본값'

    _materialize(c, mapping)
    print(f'[ROSTER] 투입 명단 ({src}): 관측 {c["observe_drone"]}, 탐색 {c["searchers"]}', flush=True)
    for line in _describe(c, mapping):
        print('  ' + line, flush=True)
