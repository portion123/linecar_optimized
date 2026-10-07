"""Client for one control-server process (see server/server.c)."""
import os, struct, subprocess
from collections import namedtuple

FIELDS = ('now state left_pwm right_pwm mask dir stop_reason running mode attempts sweep phase flags '
          'left_target right_target left_command right_command error angle left_filtered right_filtered '
          'approach goal aux0 aux1').split()
Record = namedtuple('Record', FIELDS)
_REC = struct.Struct('<IBbbBbBBBBBBB12f')

def core_bytes(raw):
    """Control-output fields shared by every variant: now,state,pwm,mask,dir,running,sweep,
    targets,commands,error,angle,filtered RPM,approach,goal.  Excludes observation-only
    fields (stop_reason, mode, attempts, phase, flags, aux0, aux1)."""
    return raw[0:9] + raw[10:11] + raw[13:14] + raw[16:56]
assert _REC.size == 64

class Controller:
    def __init__(self, binary, env=None):
        self.binary = str(binary)
        self.proc = subprocess.Popen([self.binary], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0, env=env)
        self._in, self._out = self.proc.stdin.fileno(), self.proc.stdout.fileno()
        self.raw = b''

    def _read(self, n):
        chunks = []
        while n:
            b = os.read(self._out, n)
            if not b:
                err = self.proc.stderr.read().decode(errors='replace')
                raise RuntimeError(f'server {self.binary} died: rc={self.proc.wait()} {err[-2000:]}')
            chunks.append(b); n -= len(b)
        return b''.join(chunks)

    def _call(self, payload):
        os.write(self._in, payload)
        raw = self._read(64)
        self.raw = raw
        return Record._make(_REC.unpack(raw))

    def init(self): return self._call(b'I')
    def start(self, mask): return self._call(b'S' + bytes([mask & 255]))
    def step(self, now, left, right, mask):
        left = max(-32768, min(32767, int(left))); right = max(-32768, min(32767, int(right)))
        return self._call(b'T' + struct.pack('<IhhB', now & 0xFFFFFFFF, left, right, mask & 255))
    def key(self, now, key): return self._call(b'K' + struct.pack('<IB', now & 0xFFFFFFFF, key))
    def extended(self):
        os.write(self._in, b'X')
        n = struct.unpack('<H', self._read(2))[0]
        return self._read(n) if n else b''

    def close(self):
        if self.proc.poll() is None:
            try:
                os.write(self._in, b'Q')
            except OSError:
                pass
            rc = self.proc.wait()
            err = self.proc.stderr.read().decode(errors='replace')
            if rc != 0 or 'runtime error' in err or 'AddressSanitizer' in err:
                raise RuntimeError(f'server {self.binary} exit rc={rc}: {err[-3000:]}')
            return err
        return ''
    def __enter__(self): return self
    def __exit__(self, *a): self.close()
