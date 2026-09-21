"""M3 review contract and authored JavaScript audit regressions for M4."""
import pytest

from dashboard.config import load_config
from dashboard.tests import verify_m2_constraints as audit


def test_hello_marker_contract_comes_from_bt_not_field_or_lane_inference():
    cfg = load_config(mock=True)
    cfg.bt['coshow'].update(observe_drone='renamed_reader',
                            mission_marker_ids=[31, 38], marker_id_offset=30)
    cfg.field['markers']['mission'] = [{'id': 99, 'position': [0, 0]}]
    hello = cfg.hello(True)
    assert hello.get('observe_drone') == 'renamed_reader'
    assert hello.get('mission_marker_ids') == [31, 38]
    assert hello.get('marker_id_offset') == 30
    assert len(hello) == 16


def test_missing_bt_marker_contract_is_explicit_null_without_defaults():
    cfg = load_config(mock=True)
    for key in ('observe_drone', 'mission_marker_ids', 'marker_id_offset'):
        cfg.bt['coshow'].pop(key)
    hello = cfg.hello()
    assert all(key in hello and hello[key] is None
               for key in ('observe_drone', 'mission_marker_ids', 'marker_id_offset'))


def run_audit(tmp_path, monkeypatch, source, relative='static/js/nested/fixture.js'):
    (tmp_path / 'runtime.py').write_text('import asyncio\n')
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source)
    monkeypatch.setattr(audit, 'ROOT', tmp_path)
    audit.main()


@pytest.mark.parametrize('literal', [
    'cf230', '/preflight/ready', '/coshow/mission_state', '/aideck/data',
    '/renamed/land', '/renamed/arm', '/renamed/navigate_to_pose',
    'marker_detections', 'limo_new', 'spare_', 'spare_CF07',
])
def test_recursive_javascript_audit_catches_external_strings(tmp_path, monkeypatch, literal):
    with pytest.raises(AssertionError):
        run_audit(tmp_path, monkeypatch, 'const value = ' + repr(literal) + ';')


@pytest.mark.parametrize('source', [
    r'const value = "\x63f230";',
    r'const value = "\u0063f230";',
    r'const value = "\u{63}f230";',
    'const value = `robot cf230 ${id}`;',
    'const value = `robot ${lookup("cf230")}`;',
    'const value = `robot ${`nested ${lookup("cf230")}`}`;',
    'const value = "https://local.example"; const role = "cf230";',
    'const ratio = width / height; const role = "cf230";',
    'if (ready) /["/]/.test(value); const role = "cf230";',
])
def test_javascript_lexing_catches_escaped_and_template_expression_strings(
        tmp_path, monkeypatch, source):
    with pytest.raises(AssertionError):
        run_audit(tmp_path, monkeypatch, source)


@pytest.mark.parametrize('source', [
    'return ids.filter(id => id >= 11 && id <= 14);',
    'return ids.filter(id => 14 >= id && 11 <= id);',
    'const mission_marker_ids = [11, 14];',
    'const missionMarkerIds = [11, 12, 13, 14];',
    'const missionMarkerIds = [0xb, 0xe];',
    'return ids.filter(id => id >= 1.1e1 && id <= 1_4);',
])
def test_javascript_audit_rejects_hardcoded_mission_marker_range(
        tmp_path, monkeypatch, source):
    with pytest.raises(AssertionError, match='mission marker'):
        run_audit(tmp_path, monkeypatch, source)


def test_javascript_audit_ignores_comments_regex_and_unrelated_style_numbers(
        tmp_path, monkeypatch, capsys):
    source = r'''// const bad = 'cf230'; id >= 11 && id <= 14;
    /* const bad = "/preflight/ready"; */
    const url = "https://local.example";
    const pattern = /["/]*cf230/;
    if (ready) /cf230/.test(value);
    const padding = [11, 14]; const size = {width: 11, height: 14};
    const markerDimensions = [11, 14];
    const color = '#111414'; const ratio = width / 14;
    const text = `style ${11}px ${14}px`;
    const allowed = ['spare_prefix', 'limo_status_template', 'limo_area'];
    '''
    run_audit(tmp_path, monkeypatch, source)
    output = capsys.readouterr().out
    assert '1 authored JavaScript files' in output
    assert 'string literals' in output and 'number literals' in output


def test_vendor_is_counted_but_never_interpreted_for_external_name_audit(
        tmp_path, monkeypatch, capsys):
    vendor = tmp_path / 'static/vendor/original.js'
    vendor.parent.mkdir(parents=True)
    vendor.write_text("const external = 'cf230'; const missionMarkerIds = [11,14];")
    run_audit(tmp_path, monkeypatch, "const data = 'spare_prefix';")
    assert '1 vendor JavaScript files excluded from external-name/literal audit' in capsys.readouterr().out
