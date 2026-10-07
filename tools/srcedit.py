"""Minimal-diff source editing: keep BOM and each unchanged line's CRLF/LF.

apply(path, [(old, new), ...]) : exact, unique replacements on the
newline-normalised text, then re-emit with original line endings; new or
replaced lines inherit the ending style of the lines they replace / follow.
"""
import difflib, sys
from pathlib import Path
BOM = b'\xef\xbb\xbf'

def _split(raw):
    bom = raw.startswith(BOM)
    body = raw[3:] if bom else raw
    parts = body.split(b'\n')
    flags = [p.endswith(b'\r') for p in parts]
    lines = [p[:-1] if f else p for p, f in zip(parts, flags)]
    return bom, [l.decode('utf-8') for l in lines], flags

def encode(old_raw, new_text):
    bom, old_lines, flags = _split(old_raw)
    new_lines = new_text.split('\n')
    out_flags = []
    sm = difflib.SequenceMatcher(a=old_lines, b=new_lines, autojunk=False)
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == 'equal':
            out_flags += flags[i1:i2]
        elif tag in ('replace', 'insert'):
            src = flags[i1:i2] if tag == 'replace' and i2 > i1 else ([flags[i1 - 1]] if i1 > 0 else [False])
            style = sum(src) * 2 > len(src)
            out_flags += [style] * (j2 - j1)
    # final element is the text after the last '\n' (normally ''), never gets '\r'
    out_flags[-1] = False if new_lines[-1] == '' else out_flags[-1]
    body = '\n'.join(l + ('\r' if f else '') for l, f in zip(new_lines, out_flags))
    return (BOM if bom else b'') + body.encode('utf-8')

def text(path):
    return '\n'.join(_split(Path(path).read_bytes())[1])

def apply(path, pairs):
    p = Path(path); raw = p.read_bytes(); t = text(path)
    for old, new in pairs:
        n = t.count(old)
        if n != 1: raise SystemExit(f'{path}: expected 1 match, found {n}: {old[:80]!r}')
        t = t.replace(old, new)
    p.write_bytes(encode(raw, t))

def restore(original, current):
    """Re-encode `current` (any endings) using `original`'s line-ending pattern."""
    cur = Path(current).read_bytes()
    cur_text = '\n'.join(_split(cur)[1])
    Path(current).write_bytes(encode(Path(original).read_bytes(), cur_text))

if __name__ == '__main__':
    restore(sys.argv[1], sys.argv[2])
