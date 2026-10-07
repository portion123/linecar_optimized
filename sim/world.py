"""Extended plant for the new scenario families (independent of firmware).

Geometry: tracks built from flat-capped tape strips (straight or arc) so that
corners are square (tape overlap), optional over-run stubs at corners, short
gaps, rounded corners and non-90-degree corners.  Mirror = reflect y -> -y.
Sensor: eight probes at 12 mm pitch, 175 mm ahead of the axle, each with a
round footprint (radius 3 mm); a probe reads black when the covered fraction
plus Gaussian noise exceeds its own threshold.  Optional random dropouts.
Motors: either the ORIGINAL first-order model (track_model.SquarePlant
equations) or a dynamic model (DYN) with ground load, coast at PWM 0, spin
scrub, static breakaway, optional low-speed Stribeck friction (LOW8), wheel
slip and effective track-width error.  All parameters are model assumptions,
fixed before any candidate firmware was evaluated.
"""
import math
from dataclasses import dataclass, field

TWO_PI = 2 * math.pi

def _wrap(a):
    return (a + math.pi) % TWO_PI - math.pi

class LineStrip:
    """Flat-capped tape rectangle along A->B, half width w, extended e0 before A and e1 after B."""
    def __init__(self, ax, ay, bx, by, w, e0=0.0, e1=0.0):
        L = math.hypot(bx - ax, by - ay)
        self.ux, self.uy = (bx - ax) / L, (by - ay) / L
        self.ax, self.ay = ax - self.ux * e0, ay - self.uy * e0
        self.L = L + e0 + e1; self.w = w
    def sd(self, x, y):
        dx, dy = x - self.ax, y - self.ay
        t = dx * self.ux + dy * self.uy
        n = abs(-dx * self.uy + dy * self.ux)
        qa = max(-t, t - self.L)          # along-axis outside distance
        qb = n - self.w                   # across-axis outside distance
        if qa <= 0 and qb <= 0: return max(qa, qb)
        return math.hypot(max(qa, 0.0), max(qb, 0.0))
    def mirrored(self):
        s = LineStrip.__new__(LineStrip); s.__dict__.update(self.__dict__)
        s.ay, s.uy = -self.ay, -self.uy; return s

class ArcStrip:
    """Tape along a circular arc: centre, radius R, start angle a0, signed sweep, half width w."""
    def __init__(self, cx, cy, R, a0, sweep, w):
        self.cx, self.cy, self.R, self.a0, self.sweep, self.w = cx, cy, R, a0, sweep, w
    def sd(self, x, y):
        dx, dy = x - self.cx, y - self.cy
        a = math.atan2(dy, dx)
        rel = _wrap(a - self.a0) if self.sweep > 0 else _wrap(self.a0 - a)
        span = abs(self.sweep)
        if -1e-9 <= rel <= span + 1e-9 or (rel < 0 and rel + TWO_PI <= span):
            return abs(math.hypot(dx, dy) - self.R) - self.w
        best = 1e9
        for ang in (self.a0, self.a0 + self.sweep):
            px, py = self.cx + self.R * math.cos(ang), self.cy + self.R * math.sin(ang)
            tx, ty = -math.sin(ang), math.cos(ang)
            t = abs((x - px) * tx + (y - py) * ty)
            n = abs(math.hypot(dx, dy) - self.R) - self.w
            best = min(best, math.hypot(t, max(n, 0.0)))
        return best
    def mirrored(self):
        return ArcStrip(self.cx, -self.cy, self.R, -self.a0, -self.sweep, self.w)

class Path:
    """Closed centre-line path made of ('L', ax,ay,bx,by) and ('A', cx,cy,R,a0,sweep) pieces."""
    def __init__(self, pieces):
        self.pieces = pieces; self.starts = []; s = 0.0
        for p in pieces:
            self.starts.append(s); s += self._len(p)
        self.length = s
    @staticmethod
    def _len(p):
        if p[0] == 'L': return math.hypot(p[3] - p[1], p[4] - p[2])
        return abs(p[5]) * p[3]
    def nearest(self, x, y):
        """Return (distance, along, piece index, path heading)."""
        best = (1e9, 0.0, 0, 0.0)
        for i, p in enumerate(self.pieces):
            if p[0] == 'L':
                _, ax, ay, bx, by = p; dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
                d = math.hypot(x - ax - t * dx, y - ay - t * dy)
                if d < best[0]: best = (d, self.starts[i] + t * math.sqrt(L2), i, math.atan2(dy, dx))
            else:
                _, cx, cy, R, a0, sw = p
                a = math.atan2(y - cy, x - cx)
                rel = _wrap(a - a0) if sw > 0 else _wrap(a0 - a)
                if rel < 0: rel = 0.0 if rel > -math.pi / 2 else abs(sw)
                rel = min(rel, abs(sw))
                ang = a0 + math.copysign(rel, sw)
                px, py = cx + R * math.cos(ang), cy + R * math.sin(ang)
                d = math.hypot(x - px, y - py)
                if d < best[0]: best = (d, self.starts[i] + rel * R, i, ang + math.copysign(math.pi / 2, sw))
        return best
    def mirrored(self):
        out = []
        for p in self.pieces:
            if p[0] == 'L': out.append(('L', p[1], -p[2], p[3], -p[4]))
            else: out.append(('A', p[1], -p[2], p[3], -p[4], -p[5]))
        return Path(out)

@dataclass
class Track:
    path: Path
    strips: list
    corners: int                 # number of turn features per lap (corners + arcs)
    corner_pieces: list = field(default_factory=list)  # piece index AFTER each sharp corner
    stub_lines: list = field(default_factory=list)     # (ax, ay, bx, by): tape over-run centre lines
    def mirrored(self):
        return Track(self.path.mirrored(), [s.mirrored() for s in self.strips], self.corners, list(self.corner_pieces),
                     [(ax, -ay, bx, -by) for ax, ay, bx, by in self.stub_lines])
    def metric_distance(self, x, y):
        """Lateral-error metric: distance to the tape centre line, where a corner over-run
        stub IS tape (metric correction 2026-10-07, see docs/assumptions.md section 8)."""
        d = self.path.nearest(x, y)[0]
        for ax, ay, bx, by in self.stub_lines:
            dx, dy = bx - ax, by - ay; L2 = dx * dx + dy * dy
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = min(d, math.hypot(x - ax - t * dx, y - ay - t * dy))
        return d

def polygon_track(vertices, w=0.010, stubs=None, gaps=None, arcs=None):
    """Closed polygon (CCW => left turns). stubs[i]: over-run length past vertex i (m).
    gaps: list of (edge index, start distance from edge start, gap length).
    arcs: {vertex index: radius} replaces the sharp vertex by a tangent arc."""
    n = len(vertices); stubs = stubs or [0.0] * n; gaps = gaps or []; arcs = arcs or {}
    pts = []
    for i in range(n):
        p0, p1, p2 = vertices[i - 1], vertices[i], vertices[(i + 1) % n]
        if i in arcs:
            R = arcs[i]
            h0 = math.atan2(p1[1] - p0[1], p1[0] - p0[0]); h1 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
            turn = _wrap(h1 - h0); t = R * math.tan(abs(turn) / 2)
            a = (p1[0] - t * math.cos(h0), p1[1] - t * math.sin(h0))
            b = (p1[0] + t * math.cos(h1), p1[1] + t * math.sin(h1))
            side = 1 if turn > 0 else -1
            c = (a[0] - side * R * math.sin(h0), a[1] + side * R * math.cos(h0))
            pts.append(('arc', a, b, c, R, math.atan2(a[1] - c[1], a[0] - c[0]), turn))
        else:
            pts.append(('v', p1))
    pieces, strips, corner_pieces, stub_lines = [], [], [], []
    for i in range(n):
        cur, nxt = pts[i], pts[(i + 1) % n]
        start = cur[2] if cur[0] == 'arc' else cur[1]
        end = nxt[1] if nxt[0] == 'arc' else nxt[1]
        if cur[0] == 'arc':
            _, a, b, c, R, a0, turn = cur
            pieces.append(('A', c[0], c[1], R, a0, turn)); strips.append(ArcStrip(c[0], c[1], R, a0, turn, w))
        else:
            corner_pieces.append(len(pieces))
        e0 = w if cur[0] == 'v' else 0.0
        e1 = (w + stubs[(i + 1) % n]) if nxt[0] == 'v' else 0.0
        pieces.append(('L', start[0], start[1], end[0], end[1]))
        L = math.hypot(end[0] - start[0], end[1] - start[1])
        ux, uy = (end[0] - start[0]) / L, (end[1] - start[1]) / L
        cuts = sorted((g[1], g[2]) for g in gaps if g[0] == i)
        s0, ext0 = 0.0, e0
        for gs, gl in cuts:
            strips.append(LineStrip(start[0] + ux * s0, start[1] + uy * s0, start[0] + ux * gs, start[1] + uy * gs, w, ext0, 0.0))
            s0, ext0 = gs + gl, 0.0
        strips.append(LineStrip(start[0] + ux * s0, start[1] + uy * s0, end[0], end[1], w, ext0, e1))
        if nxt[0] == 'v' and stubs[(i + 1) % n] > 0:
            st = stubs[(i + 1) % n]
            stub_lines.append((end[0], end[1], end[0] + ux * st, end[1] + uy * st))
    return Track(Path(pieces), strips, n, corner_pieces, stub_lines)

# ---------------- motors ----------------
@dataclass
class MotorParams:
    gain: float = 20.4          # RPM per % PWM above friction (free running)
    dead: float = 6.48          # free-running deadband % PWM
    load: float = 0.0           # extra ground Coulomb friction % PWM (DYN)
    breakaway: float = 14.0     # static breakaway % PWM
    tau: float = 0.12           # response time constant (s)
    spin_load: float = 0.0      # extra Coulomb friction while wheels counter-rotate (% PWM)
    stribeck: float = 0.0       # extra friction at zero speed, fading out by stribeck_rpm (LOW8)
    stribeck_rpm: float = 8.0
    coast: bool = False         # True: PWM 0 = TB6612 coast (friction-only decel)
    slip_spin: float = 0.0      # ground slip fraction while spinning
    slip_fwd: float = 0.0

class Wheel:
    def __init__(self, p):
        self.p = p; self.rpm = 0.0
    def step_original(self, pwm, dt):
        p = self.p; mag = min(100.0, abs(float(pwm))); sign = -1.0 if pwm < 0 else 1.0
        if abs(self.rpm) < 0.8 and mag < p.breakaway: eq = 0.0
        else: eq = sign * p.gain * max(0.0, mag - p.dead)
        self.rpm += (1.0 - math.exp(-dt / p.tau)) * (eq - self.rpm)
    def friction(self, spin):
        p = self.p; f = p.dead + p.load + (p.spin_load if spin else 0.0)
        if p.stribeck: f += p.stribeck * max(0.0, 1.0 - abs(self.rpm) / p.stribeck_rpm)
        return f
    def step_dynamic(self, pwm, dt, spin):
        """dw/dt = (g/tau) * (V - w/g - F sign(w)); PWM 0 coasts (no back-EMF term)."""
        p = self.p; V = max(-100.0, min(100.0, float(pwm))); k = p.gain / p.tau
        sub = 5; h = dt / sub
        for _ in range(sub):
            F = self.friction(spin)
            if abs(self.rpm) < 0.5:
                drive = V
                if abs(drive) <= max(p.breakaway, F): self.rpm = 0.0; continue
                self.rpm = math.copysign(0.5, drive)
            emf = 0.0 if (V == 0.0 and p.coast) else self.rpm / p.gain
            acc = k * (V - emf - math.copysign(F, self.rpm))
            new = self.rpm + acc * h
            if new * self.rpm < 0 and abs(V) <= F: new = 0.0
            self.rpm = new

class WorldPlant:
    """Car on a Track. dynamic=False uses the original motor equations."""
    def __init__(self, track, left, right, dynamic=False, start_along=0.12, offset=0.0, heading=0.0,
                 wheel_d=0.044, track_w=0.133, track_w_true=None, front=0.175, pitch=0.012,
                 cpr=(251.0, 265.35), footprint=0.003, thresholds=None, sensor_noise=0.0,
                 dropout=0.0, rng=None):
        self.track = track; self.wheels = [Wheel(left), Wheel(right)]; self.dynamic = dynamic
        self.wheel_d, self.track_w = wheel_d, (track_w_true or track_w); self.front, self.pitch = front, pitch
        self.cpr = cpr; self.footprint = footprint
        self.thresholds = thresholds or [0.5] * 8; self.sensor_noise = sensor_noise; self.dropout = dropout
        self.rng = rng
        p0 = track.path.pieces[0]
        h = math.atan2(p0[4] - p0[2], p0[3] - p0[1])
        self.x = p0[1] + start_along * math.cos(h) - offset * math.sin(h)
        self.y = p0[2] + start_along * math.sin(h) + offset * math.cos(h)
        self.heading = h + heading
        self.residual = [0.0, 0.0]; self.time_s = 0.0; self.travel = 0.0
        self.prev_along = track.path.nearest(self.x, self.y)[1]; self.initial_heading = self.heading
        self.wheel_dist = [0.0, 0.0]
    @property
    def laps(self): return self.travel / self.track.path.length
    def probe_points(self):
        c, s = math.cos(self.heading), math.sin(self.heading)
        fx, fy = self.x + self.front * c, self.y + self.front * s
        return [(fx - (3.5 - i) * self.pitch * s, fy + (3.5 - i) * self.pitch * c) for i in range(8)]
    def coverage(self, x, y):
        sd = min(st.sd(x, y) for st in self.track.strips)
        return max(0.0, min(1.0, 0.5 - sd / (2 * self.footprint)))
    def sensor_mask(self, clean=False):
        """clean=True: the noise-free reading used once when KEY1 starts the run."""
        mask = 0
        for i, (x, y) in enumerate(self.probe_points()):
            c = self.coverage(x, y)
            if not clean and self.sensor_noise and 0.0 < c < 1.0: c += self.rng.gauss(0.0, self.sensor_noise)
            if c >= self.thresholds[i]:
                if clean or not (self.dropout and self.rng.random() < self.dropout): mask |= 1 << i
        return mask
    def front_error(self):
        c, s = math.cos(self.heading), math.sin(self.heading)
        return (self.track.metric_distance(self.x + self.front * c, self.y + self.front * s),)
    def advance(self, lpwm, rpwm, dt=0.02):
        old = [w.rpm for w in self.wheels]
        spin = (lpwm > 0 > rpwm) or (lpwm < 0 < rpwm)
        for w, pwm in zip(self.wheels, (lpwm, rpwm)):
            if self.dynamic: w.step_dynamic(pwm, dt, spin)
            else: w.step_original(pwm, dt)
        rpm = [(a + w.rpm) * 0.5 for a, w in zip(old, self.wheels)]
        circ = math.pi * self.wheel_d
        counts = []
        for i in range(2):
            self.residual[i] += rpm[i] * self.cpr[i] * dt / 60.0
            k = int(self.residual[i]); self.residual[i] -= k; counts.append(k)
        v = [r * circ / 60.0 for r in rpm]
        counter = v[0] * v[1] < 0
        slip = [(w.p.slip_spin if counter else w.p.slip_fwd) for w in self.wheels]
        v = [vi * (1.0 - si) for vi, si in zip(v, slip)]
        lin = (v[0] + v[1]) * 0.5; ang = (v[1] - v[0]) / self.track_w; dh = ang * dt
        if abs(ang) < 1e-10:
            self.x += lin * dt * math.cos(self.heading); self.y += lin * dt * math.sin(self.heading)
        else:
            R = lin / ang
            self.x += R * (math.sin(self.heading + dh) - math.sin(self.heading))
            self.y -= R * (math.cos(self.heading + dh) - math.cos(self.heading))
        self.heading += dh; self.time_s += dt
        along = self.track.path.nearest(self.x, self.y)[1]; Lp = self.track.path.length
        d = (along - self.prev_along + Lp / 2) % Lp - Lp / 2
        self.travel += d; self.prev_along = along
        return counts
