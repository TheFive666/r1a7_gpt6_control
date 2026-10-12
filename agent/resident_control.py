"""Persistent single-action inbox; callers never own the hardware lifetime."""
import json
from pathlib import Path


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False))
    temporary.replace(path)


class ResidentInbox:
    def __init__(self, directory, execute, freeze):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.execute = execute
        self.freeze = freeze
        self.halted = True  # A new service starts HOLD, never resumes a recipe.

    def halt(self):
        self.halted = True
        self.freeze()

    def poll(self):
        for path in sorted(self.directory.glob('*.request.json')):
            result = path.with_name(path.name.replace('.request.json', '.result.json'))
            if result.exists():
                continue  # Persistent at-most-once request identity.
            claimed = path.with_name(path.name.replace('.request.json', '.claimed'))
            if claimed.exists():
                atomic_json(result, dict(ok=False, error='interrupted request; never replayed'))
                self.halt()
                continue
            claimed.touch(exist_ok=False)
            try:
                request = json.loads(path.read_text())
                action = request['action']
                if action == 'stop':
                    self.halt()
                    response = dict(holding=True)
                elif action == 'resume':
                    # Hardware-specific validation is required before unlatching.
                    response = self.execute(request)
                    self.halted = False
                elif action == 'observe':
                    response = self.execute(request)
                elif self.halted:
                    raise RuntimeError('HOLD latched; explicit resume required')
                else:
                    response = self.execute(request)
                atomic_json(result, dict(ok=True, **(response or {})))
            except BaseException as exc:
                self.halt()
                atomic_json(result, dict(ok=False, error=str(exc), holding=True))
