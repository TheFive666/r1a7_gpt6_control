#!/usr/bin/env python3
"""Submit one request to the resident owner; no hardware imports or publishers."""
import argparse
import json
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('request', help='JSON action')
p.add_argument('--view-dir', type=Path, default=Path('logs/fixed_3cm_view'))
a = p.parse_args()
inbox = a.view_dir / 'inbox'
if not inbox.is_dir():
    raise SystemExit('resident service inbox unavailable')
name = str(time.time_ns())
request = inbox / f'{name}.request.json'
tmp = inbox / f'{name}.tmp'
tmp.write_text(json.dumps(json.loads(a.request), ensure_ascii=False))
tmp.replace(request)
result = inbox / f'{name}.result.json'
progress = set()
deadline = time.monotonic() + 20
while not result.exists():
    try:
        status = json.loads((a.view_dir / 'status.json').read_text())
        if status.get('planning_phase') == 'executing':
            progress.add(round(status.get('execution_progress', 0), 2))
    except (OSError, ValueError):
        pass
    if time.monotonic() > deadline:
        raise SystemExit(f'No result yet; owner remains alive. Inspect {result}')
    time.sleep(.03)
print(result.read_text())
print('execution_progress_samples:', sorted(progress))
