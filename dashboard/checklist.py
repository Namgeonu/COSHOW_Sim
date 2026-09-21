"""Pure checklist gates over the dashboard's received observations.

No ROS imports, graph queries, environment reads, or process operations belong
here. The adapter and server provide those facts in ``state`` and ``context``.
"""
import math


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _fresh(age, limit):
    return _number(age) and _number(limit) and 0.0 <= age < limit


def _pair(value):
    return isinstance(value, (tuple, list)) and len(value) == 2 and all(_number(v) for v in value)


def _node_identity(value):
    parts = str(value or '').strip('/').split('/')
    return parts[-1], '/' + '/'.join(parts[:-1]) if len(parts) > 1 else '/'


def _row(identifier, group, label, ok, blocking=True, detail='', status=None):
    return {'id': identifier, 'group': group, 'label': label, 'ok': bool(ok),
            'blocking': bool(blocking), 'detail': str(detail),
            'status': status or ('pass' if ok else 'fail')}


def external_observation(kind, cfg, context, now):
    """Count foreign nodes and recent JSON activity, allowing our DDS residue.

    ``active`` is the receipt-only signal used by the visitor observation mode.
    A retained sample alone cannot make that signal true.
    """
    process = context.get('processes', {}).get(kind, {})
    alive = process.get('alive') is True
    exited = process.get('exited_at')
    grace = cfg.raw.get('external_node_grace_s', 20.0)
    in_grace = (not alive and _number(exited) and _number(grace)
                and 0.0 <= now - exited <= grace)
    expected = 1 if alive or in_grace else 0
    identity = _node_identity(cfg.raw.get('nodes', {}).get(kind))
    count = sum(tuple(item) == identity for item in context.get('nodes', []))
    receipts = context.get('receipts', {})

    def recent(key):
        return [value for value in receipts.get(key, [])
                if _number(value) and now - 3.0 < value <= now
                and (not _number(exited) or value > exited)]

    if kind == 'bt':
        received = recent('mission')
    else:
        received = max(recent('preflight_status'), recent('preflight_ready'), key=len)
    active = not alive and len(received) >= 2
    external = count > expected or active
    if external:
        detail = '터미널에서 띄운 {}를 먼저 종료하라 (노드 {}/{}, 최근 수신 {}건)'.format(
            kind, count, expected, len(received))
    elif in_grace and count:
        detail = '이전 실행 노드 잔류 {:.1f}초'.format(now - exited)
    else:
        detail = '외부 인스턴스 없음 (노드 {}/{})'.format(count, expected)
    return {'external': external, 'active': active, 'detail': detail,
            'count': count, 'expected': expected}


def evaluate(state, cfg, context, now):
    """Return every role safety gate plus advisory fleet observations.

    Only rows with ``blocking`` participate in start eligibility. Spare pose,
    camera, navigation and arm requirements are explicitly skipped. Unknown
    measurements never pass a blocking gate.
    """
    result = []
    raw = cfg.raw
    freshness = raw.get('freshness_s', {})
    battery = raw.get('battery_v', {})
    preflight = state.get('preflight') or {}
    reports = preflight.get('drones') or {}
    checking = (state.get('run') or {}).get('state') == 'CHECKING'
    monitor_alive = context.get('processes', {}).get('preflight', {}).get('alive') is True
    monitor_ok = (preflight.get('ready') is True and preflight.get('abort_reason') is None
                  and _fresh(preflight.get('age'), 2.0) and monitor_alive)

    def add(name, key, label, ok, blocking=True, detail='', status=None):
        result.append(_row(name + '.' + key, name, label, ok, blocking, detail, status))

    def age_detail(age):
        return '{:.2f}s 전'.format(age) if _number(age) and age >= 0 else '정보 없음'

    def voltage_row(name, robot, role, drone):
        voltage = robot.get('battery_v')
        limits = battery if drone else raw.get('limo_battery_v', {})
        if not _number(voltage):
            add(name, 'battery', '배터리', False, role and drone, '정보 없음', 'unavailable')
            return
        detail = '{:.2f} V'.format(voltage)
        block, warn = limits.get('block'), limits.get('warn')
        if not _number(block):
            add(name, 'battery', '배터리', False, role and drone,
                detail + ' · 임계값 미설정', 'unavailable')
            return
        ok = voltage >= block
        status = 'warning' if (not ok and not role) or (ok and _number(warn) and voltage < warn) else None
        add(name, 'battery', '배터리', ok, role and drone, detail, status)

    for name, metadata in cfg.robots.items():
        robot = (state.get('robots') or {}).get(name) or {}
        role = metadata.get('role') is not None
        drone = metadata.get('kind') == 'drone'
        if drone:
            status_age = robot.get('status_age')
            add(name, 'radio', '라디오 링크', _fresh(status_age, freshness.get('status', 1.0)),
                role, age_detail(status_age))
        voltage_row(name, robot, role, drone)
        pose_age = robot.get('pose_age')
        pose_limit = freshness.get('pose' if drone else 'odom', .8 if drone else 1.0)
        add(name, 'pose', '위치 수신', _fresh(pose_age, pose_limit) if role else True,
            role, age_detail(pose_age) if role else '스페어 위치는 시작 조건에서 제외',
            None if role else 'skipped')
        if drone:
            if role:
                report = reports.get(name) or {}
                gate_ok = all(report.get(key) is True for key in ('pose_ok', 'kal_ok', 'sup_ok'))
                add(name, 'preflight', '베이스 위치·칼만·슈퍼바이저', gate_ok,
                    detail='진행 중' if checking and not gate_ok else ('관문 통과' if gate_ok else '관문 미통과'),
                    status='pending' if checking and not gate_ok else None)
                # Put the live monitor above the historical arm-stage display.
                add(name, 'monitor', '안전 감시 생존', monitor_ok,
                    detail=preflight.get('abort_reason') or
                    ('감시 중' if monitor_ok else 'ready·상태 신선도·소유 프로세스 생존 확인 필요'))
                add(name, 'armed', '무장', robot.get('armed') is True and robot.get('can_fly') is True,
                    detail='실시간 무장·비행 가능 비트')
                camera = robot.get('camera') or {}
                fps = camera.get('fps')
                minimum = raw.get('camera_min_fps', 5.0)
                camera_ok = (camera.get('stream_ok') is True and _number(fps)
                             and _number(minimum) and fps >= minimum)
                add(name, 'camera', '카메라', camera_ok,
                    detail='{:.1f} fps'.format(fps) if _number(fps) else '정보 없음')
            else:
                for key, label in (('preflight', '베이스 위치·칼만·슈퍼바이저'),
                                   ('monitor', '안전 감시 생존'), ('armed', '무장 요구'),
                                   ('camera', '카메라')):
                    add(name, key, label, True, False, '스페어는 제어·카메라 대상 아님', 'skipped')
                fresh_status = _fresh(robot.get('status_age'), freshness.get('status', 1.0))
                armed = robot.get('armed')
                known = fresh_status and isinstance(armed, bool)
                add(name, 'spare_unarmed', '스페어 비무장', known and not armed, False,
                    '스페어 무장 감지' if known and armed else ('비무장' if known else '정보 없음'),
                    'warning' if known and armed else ('pass' if known else 'unavailable'))
        else:
            add(name, 'nav', 'Nav2', robot.get('nav_ready') is True if role else True,
                role, '액션 서버 준비' if role else '스페어 액션 서버는 검사하지 않음',
                None if role else 'skipped')
        ping_state = robot.get('ping') or {}
        ip = ping_state.get('ip', metadata.get('ip'))
        add(name, 'ping', 'AI Deck ping' if drone else 'ping',
            bool(ip) and ping_state.get('ok') is True, role,
            ('{} · {}'.format(ip, '응답 정상' if ping_state.get('ok') is True else '응답 없음'))
            if ip else '미설정', 'unavailable' if not ip else None)

    def global_row(key, label, ok, detail='', blocking=True, status=None):
        add('global', key, label, ok, blocking, detail, status)

    node_list = [tuple(item) for item in context.get('nodes', [])]
    for key, label in (('server', '드론 서버 노드'), ('aideck', '카메라 노드')):
        configured = raw.get('nodes', {}).get(key)
        global_row(key, label, bool(configured) and _node_identity(configured) in node_list,
                   configured or '설정 없음')
    stack = state.get('stack') or {}
    external_stack = any(stack.get(key) == 'external' for key in ('crazyflie_server', 'aideck'))
    roster_hash, applied_hash = stack.get('roster_hash'), stack.get('applied_hash')
    matching = bool(roster_hash) and bool(applied_hash) and roster_hash == applied_hash
    global_row('roster', '스택 로스터 일치', matching and not external_stack,
               '외부 스택 · 확인 불가' if external_stack else
               ('로스터 일치' if matching else '스택 재기동 필요'),
               not external_stack, 'skipped' if external_stack else None)
    counts = stack.get('radio_counts')
    maximum = raw.get('fleet', {}).get('max_per_radio', 5)
    radio_ok = (isinstance(counts, dict) and bool(counts) and _number(maximum) and maximum >= 1
                and all(_number(count) and 0 <= count <= maximum for count in counts.values()))
    global_row('radios', '라디오당 기체 수', radio_ok,
               ', '.join('{}: {}대'.format(key, value) for key, value in sorted(counts.items()))
               if isinstance(counts, dict) and counts else '설정 없음')
    bt = cfg.bt or {}
    coshow = bt.get('coshow') or {}
    bt_path = str(cfg.bt_path)
    search = coshow.get('search') or {}
    searchers = coshow.get('searchers') or []

    def setting(group, key, label, path, value, valid):
        add(group, key, label, valid, True,
            '{}: {}={}'.format(bt_path, path, value) if valid
            else '{}: {} 설정 없음 또는 형식 오류'.format(bt_path, path),
            None if valid else 'unavailable')

    for name, metadata in cfg.robots.items():
        if metadata.get('role') is None:
            continue
        section = 'drones' if metadata.get('kind') == 'drone' else 'limos'
        base = ((coshow.get(section) or {}).get(name) or {}).get('base')
        setting(name, 'base_config', '베이스 설정', 'coshow.{}.{}.base'.format(section, name),
                base, _pair(base))
        if name in searchers:
            zone = (search.get('zones') or {}).get(name) or {}
            valid = isinstance(zone, dict) and all(_pair(zone.get(axis)) for axis in ('x', 'y'))
            setting(name, 'search_config', '탐색 구역 설정', 'coshow.search.zones.' + name, zone, valid)
    tolerances = coshow.get('tolerances') or {}
    valid_tolerances = all(_number(tolerances.get(key)) and tolerances[key] >= 0
                           for key in ('landed_z', 'drone_at', 'limo_at'))
    setting('global', 'bt_tolerances', 'BT 허용오차 설정', 'coshow.tolerances',
            tolerances, valid_tolerances)
    land_duration = (coshow.get('durations') or {}).get('land')
    setting('global', 'bt_durations', 'BT 착륙 시간 설정', 'coshow.durations.land',
            land_duration, _number(land_duration) and land_duration > 0)
    spacing = search.get('lane_spacing')
    valid_search = (isinstance(searchers, list) and bool(searchers)
                    and _number(spacing) and spacing > 0 and search.get('lane_axis', 'y') in ('x', 'y'))
    setting('global', 'bt_search', 'BT 탐색 설정', 'coshow.search', search, valid_search)
    observe_point = coshow.get('observe_point') or {}
    valid_observe = all(_number(observe_point.get(axis)) for axis in ('x', 'y'))
    setting('global', 'bt_observe', 'BT 관측점 설정', 'coshow.observe_point', observe_point, valid_observe)
    rescue_sec = coshow.get('rescue_sec')
    setting('global', 'bt_rescue', 'BT 구조 시간 설정', 'coshow.rescue_sec',
            rescue_sec, _number(rescue_sec) and rescue_sec >= 0)
    for key, label, value in (
            ('preflight_required', '비행 전 점검 필수', (coshow.get('preflight') or {}).get('required')),
            ('emergency_land', '종료 시 비상 착륙', coshow.get('emergency_land_on_exit'))):
        global_row(key, label, value is True, '{}: {}={}'.format(bt_path, key, value))
    visualiser = ((bt.get('bt_runner') or {}).get('bt_visualiser') or {}).get('enabled')
    global_row('bt_visualiser', 'BT 시각화 창 끄기', visualiser is False,
               '{}: bt_visualiser.enabled={}'.format(bt_path, visualiser), blocking=False,
               status='pass' if visualiser is False else 'warning')
    env = context.get('env', raw.get('commands', {}).get('env', {})) or {}
    global_row('bt_environment', 'BT 실행 환경', bool(env.get('DISPLAY')) or env.get('SDL_VIDEODRIVER') == 'dummy',
               'DISPLAY 또는 SDL_VIDEODRIVER=dummy 필요')
    for kind, label in (('preflight', '외부 preflight 없음'), ('bt', '외부 BT 없음')):
        observation = external_observation(kind, cfg, context, now)
        global_row('external_' + kind, label, not observation['external'], observation['detail'])
    orphans = list(context.get('orphans') or []) + list(context.get('stack_orphans') or [])
    recovered = context.get('stack_recovered') or []
    detail = '{}개 감지 (BT/preflight·스택 포함)'.format(len(orphans))
    if recovered:
        detail += ' · 스택 {}개 재입양'.format(len(recovered))
    global_row('orphans', '고아 프로세스 없음', not orphans, detail)
    global_row('field_config', '필드 설정 로드', cfg.field_ok, '필드 설정 정상' if cfg.field_ok else '설정 없음')
    global_row('dashboard_config', '대시보드 설정 로드', cfg.dashboard_ok,
               '대시보드 설정 정상' if cfg.dashboard_ok else '설정 없음')
    for index, error in enumerate(cfg.errors):
        global_row('config_error_' + str(index), '필수 설정 오류', False, error)
    for index, warning in enumerate(getattr(cfg, 'warnings', [])):
        global_row('config_warning_' + str(index), '인벤토리 설정 미확인', False, warning,
                   blocking=False, status='warning')
    for index, error in enumerate(context.get('interface_errors', [])):
        global_row('interface_error_' + str(index), '인터페이스 설정 오류', False,
                   '{}.{}: {}'.format(error['kind'], error['key'], error['reason']))
    global_row('ping_tool', 'ping 실행 파일', context.get('ping_available') is True,
               '사용 가능' if context.get('ping_available') is True else 'ping 실행 파일 없음 또는 미확인')
    return result
