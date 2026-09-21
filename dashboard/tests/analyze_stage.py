#!/usr/bin/env python3
"""Summarize stage_probe JSONL phase, run-state, and emergency-event timelines."""
import argparse
import json
import re


SAFETY_EVENT = re.compile(r'LANDING|ABORTED|SIGINT|SIGKILL|\bland\b|\barm\b|estop|reset|착륙|취소|무장|리셋', re.I)


def analyze_lines(lines):
    timeline, events, recovery = [], set(), []
    previous_phase, previous_run, reset_at = None, None, None
    states, complete = 0, 0
    for index, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
            if not isinstance(record, dict):
                raise ValueError('JSON object required')
            message = record.get('message', record)
            if not isinstance(message, dict):
                raise ValueError('message object required')
        except (ValueError, TypeError) as exc:
            raise ValueError('Invalid JSONL line {}: {}'.format(index, exc)) from exc
        if message.get('type') != 'state':
            continue
        states += 1
        stamp, clock = record.get('received_at', ''), record.get('monotonic_s')
        def append(kind, detail, event_t=None):
            timeline.append(dict(received_at=stamp, monotonic_s=clock, event_t=event_t, kind=kind, detail=detail))
        phase = (message.get('mission') or {}).get('phase')
        run = (message.get('run') or {}).get('state')
        if phase != previous_phase:
            previous_phase = phase
            if phase:
                append('phase', phase)
                complete += phase == 'done'
        if run != previous_run:
            append('run', run)
            if run == 'IDLE' and previous_run is not None:
                reset_at = clock
            elif run == 'READY' and reset_at is not None and type(clock) in (int, float):
                recovery.append(clock - reset_at)
                reset_at = None
            previous_run = run
        for event in message.get('events', []):
            key = (event.get('t'), event.get('level'), event.get('text'))
            if key in events:
                continue
            events.add(key)
            append('safety' if SAFETY_EVENT.search(str(event.get('text', ''))) else 'event', event.get('text', ''), event.get('t'))
    return dict(states=states, completed_runs=complete, recovery_to_ready_seconds=recovery, timeline=timeline)


def render_report(result):
    lines = ['states={} completed_runs={}'.format(result['states'], result['completed_runs']),
             'received_at\tmonotonic_s\tevent_t\tkind\tdetail']
    for row in result['timeline']:
        lines.append('\t'.join(str(row[key]) for key in ('received_at', 'monotonic_s', 'event_t', 'kind', 'detail')))
    lines.append('RESET_TO_READY_SECONDS ' + json.dumps(result['recovery_to_ready_seconds']))
    lines.append('Rehearsal acceptance requires operator confirmation of landing, LIMO stop, and core-file absence.')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', help='JSONL written by stage_probe --record')
    parser.add_argument('--json', action='store_true', help='emit a machine-readable analysis')
    args = parser.parse_args()
    with open(args.recording, encoding='utf-8') as stream:
        result = analyze_lines(stream)
    print(json.dumps(result, ensure_ascii=False, indent=2) if args.json else render_report(result))


if __name__ == '__main__':
    main()
