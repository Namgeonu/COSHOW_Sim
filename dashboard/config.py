"""YAML configuration and static protocol data, with no ROS imports."""
import copy
from pathlib import Path
import shlex

import yaml

from dashboard.lanes import lanes_from_config

HERE = Path(__file__).resolve().parent


def read_yaml(path):
    value = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError('YAML mapping required: ' + str(path))
    return value


class Config:
    def __init__(self, path=None, field_path=None, mock=False, roster_override=None):
        self.mock = bool(mock)
        self.path = Path(path or HERE / 'config/dashboard.yaml').expanduser().resolve()
        self.errors, self.warnings = [], []
        self.raw = self._read(self.path)
        self.dashboard_ok = bool(self.raw)
        for key in ('robots', 'fleet', 'commands', 'topics', 'services', 'actions', 'nodes', 'types'):
            if not isinstance(self.raw.get(key), dict):
                self.errors.append(key + ': 필수 설정 블록 누락 또는 mapping 아님')
                self.raw[key] = {}
        self.field_path = Path(field_path or self.path.with_name('field.yaml')).resolve()
        self.field = self._read(self.field_path)
        self.field_ok = bool(self.field)
        self.drones = list(self.raw.get('robots', {}).get('drones', []))
        self.limos = list(self.raw.get('robots', {}).get('limos', []))
        if len(self.drones) > 256:
            self.errors.append('카메라 인덱스는 1 byte: 역할 드론은 최대 256대')
        if len(set(self.drones + self.limos)) != len(self.drones + self.limos):
            self.errors.append('역할 이름 중복')
        commands = self.raw.get('commands', {})
        self.bt_cwd = self.resolve(commands.get('bt_cwd', '../bt'))
        self.bt_path = self.bt_cwd / '__missing_config__'
        try:
            tokens = shlex.split(commands.get('bt', ''))
            value = next((t.split('=', 1)[1] for t in tokens if t.startswith('--config=')), None)
            if value is None:
                value = tokens[tokens.index('--config') + 1]
            path_value = Path(value).expanduser()
            self.bt_path = (self.bt_cwd / path_value).resolve()
        except (ValueError, IndexError):
            self.errors.append('commands.bt: --config 설정 없음')
        self.bt = self._read(self.bt_path)
        if mock:
            # In-memory fixtures only; the actual collaborator BT YAML is unchanged.
            self.bt.setdefault('coshow', {}).setdefault('preflight', {})['required'] = True
            self.bt.setdefault('bt_runner', {}).setdefault('bt_visualiser', {})['enabled'] = False
            for i, item in enumerate(self.raw.get('fleet', {}).get('drones', [])):
                item['uri'] = 'radio://{}/80/2M/{:010X}'.format(i // 4, i + 1)
                item['aideck_ip'] = item.get('aideck_ip') or '127.0.0.1'
            for item in self.raw.get('fleet', {}).get('limos', []):
                item['ip'] = item.get('ip') or '127.0.0.1'
        self.roster = {}
        self.robots = {}
        self.radio_counts = {}
        self._inventory(mock, roster_override)

    def resolve(self, value):
        # Paths in dashboard.yaml are relative to dashboard/, not shell cwd.
        return (HERE / Path(value).expanduser()).resolve()

    def _read(self, path):
        try:
            return read_yaml(path)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.errors.append(str(path) + ': ' + str(exc))
            return {}

    def _inventory(self, mock, roster_override=None):
        fleet = self.raw.get('fleet', {})
        drones = fleet.get('drones', [])
        roster_path = None
        if roster_override is None and not mock:
            try:
                value = self.raw.get('roster_file', 'run/roster.yaml')
                if not isinstance(value, str) or not value.strip():
                    raise ValueError('비어 있지 않은 파일 경로가 필요합니다')
                roster_path = self.resolve(value)
            except (OSError, TypeError, ValueError) as exc:
                self.errors.append('roster_file: ' + str(exc))
        if roster_override is not None:
            self.roster = copy.deepcopy(roster_override)
        elif mock:
            self.roster = {role: item['id'] for role, item in zip(self.drones, drones)}
        elif roster_path is not None and roster_path.exists():
            self.roster = self._read(roster_path)
        elif self.raw.get('crazyflies_template'):
            template = self._read(self.resolve(self.raw['crazyflies_template']))
            by_uri = {d.get('uri'): d['id'] for d in drones if d.get('uri')}
            self.roster = {name: by_uri[r['uri']] for name, r in template.get('robots', {}).items()
                           if name in self.drones and r.get('uri') in by_uri}
        drone_ids = [d['id'] for d in drones]
        if len(drone_ids) != len(set(drone_ids)):
            self.errors.append('fleet.drones id 중복')
        assigned = list(self.roster.values())
        if len(assigned) != len(set(assigned)):
            self.errors.append('로스터에 동일 물리 기체 중복 배정')
        for role, physical in self.roster.items():
            if role not in self.drones or physical not in drone_ids:
                self.errors.append('로스터 배정 없음: ' + str(role) + ' ← ' + str(physical))
        prefix = self.raw.get('spare_prefix')
        if not isinstance(prefix, str) or not prefix:
            self.errors.append('spare_prefix: 필수 설정 없음')
            prefix = ''
        network = self.raw.get('network', {})
        for item in drones:
            role = next((n for n in self.drones if self.roster.get(n) == item['id']), None)
            name = role or prefix + item['id']
            self.robots[name] = dict(kind='drone', role=role, fleet_id=item['id'],
                                     ip=item.get('aideck_ip') or network.get('aideck_ips', {}).get(name),
                                     uri=item.get('uri'))
            uri = item.get('uri')
            if isinstance(uri, str) and uri.startswith('radio://') and len(uri.split('/')) == 6:
                radio = uri.split('/')[2]
                self.radio_counts[radio] = self.radio_counts.get(radio, 0) + 1
            else:
                (self.errors if role or not (uri is None or isinstance(uri, str) and not uri.strip())
                 else self.warnings).append(
                    'fleet ' + item['id'] + ': radio URI 미설정 또는 형식 오류')
            if not self.robots[name]['ip']:
                (self.errors if role else self.warnings).append('fleet ' + item['id'] + ': AI Deck IP 미설정')
        for role in self.drones:
            if role not in self.robots:
                self.robots[role] = dict(kind='drone', role=role, fleet_id=None, uri=None,
                                         ip=network.get('aideck_ips', {}).get(role))
                self.errors.append('로스터 역할 미배정: ' + role)
        for item in fleet.get('limos', []):
            name = item.get('namespace')
            if not name or name in self.robots:
                self.errors.append('fleet.limos namespace 누락 또는 중복: ' + str(name))
                continue
            self.robots[name] = dict(kind='limo', role=name if name in self.limos else None,
                                     fleet_id=item['id'], ip=item.get('ip') or network.get('limo_ips', {}).get(name))
        for name, meta in self.robots.items():
            if meta['kind'] == 'limo' and not meta['ip']:
                (self.errors if meta['role'] else self.warnings).append(
                    'fleet ' + str(meta['fleet_id']) + ' (' + name + '): LIMO IP 미설정')
        for role in self.limos:
            if role not in self.robots:
                self.robots[role] = dict(kind='limo', role=role, fleet_id=None,
                                         ip=network.get('limo_ips', {}).get(role))
                self.errors.append('LIMO 역할 namespace 미설정: ' + role)

    def hello(self, mock=False):
        config = self.bt.get('coshow', {})
        bases = {n: v['base'] for kind in ('drones', 'limos')
                 for n, v in config.get(kind, {}).items() if 'base' in v}
        lanes = {}
        # One missing searcher's key must not hide other correctly configured lanes.
        for name in config.get('searchers', []):
            fragment = copy.deepcopy(self.bt)
            fragment['coshow']['searchers'] = [name]
            try:
                lanes.update(lanes_from_config(fragment))
            except (KeyError, TypeError, ValueError, ZeroDivisionError):
                pass
        point = config.get('observe_point', {})
        return dict(type='hello', mock=mock, drones=self.drones, limos=self.limos,
                    field=self.field, bases=bases, lanes=lanes,
                    observe_point=[point['x'], point['y']] if 'x' in point and 'y' in point else None,
                    observe_drone=config.get('observe_drone'),
                    mission_marker_ids=config.get('mission_marker_ids'),
                    marker_id_offset=config.get('marker_id_offset'),
                    rescue_sec=config.get('rescue_sec'),
                    display_names=self.raw.get('display', {}).get('names', {}),
                    idle_lines=self.raw.get('display', {}).get('idle_lines', []),
                    freshness_s=self.raw.get('freshness_s', {}),
                    camera_min_fps=self.raw.get('camera_min_fps'))


def load_config(path=None, field_path=None, mock=False, roster_override=None):
    return Config(path, field_path, mock, roster_override)
