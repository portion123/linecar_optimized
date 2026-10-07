"""Extract the read-only baselines from the original archives.

Sources (all inputs are original user/delivery archives, never edited):
  fix4      : user upload 365e0e0b-*FIX4*1.zip   (FIX4 real-car lapping firmware)
  fix4_p1p5 : user upload c00568ae-FIX4_P1-P5*.zip (FIX4 P1-P5 side branch)
  opt       : fix4_baseline.tar.gz inside FIX5 zip (linecar_optimized 89bafa2)
  fix5      : linecar_FIX5.zip on branch fix5_delivery (FIX5 final)
Large generated traces / git bundles are omitted from the tree but every file of
the original archive is listed with its SHA256 in MANIFEST.json, so omissions
are explicit and verifiable against the original archive.
"""
import hashlib, io, json, os, stat, sys, tarfile, zipfile
from pathlib import Path

OMIT_SUFFIX = ('geometry_traces.json', '.bundle', 'changes_from_original.patch', 'fix4_baseline.tar.gz')

def sha(data): return hashlib.sha256(data).hexdigest()

def write_tree(name, members, out_root, archive_path, archive_sha, strip):
    root = out_root / name
    if root.exists():
        sys.exit(f'{root} exists; baselines are write-once')
    manifest = {'archive': str(archive_path.name), 'archive_sha256': archive_sha,
                'files': {}, 'omitted': []}
    for rel, data in members:
        rel = rel[len(strip):] if rel.startswith(strip) else rel
        if not rel or rel.endswith('/'):
            continue
        manifest['files'][rel] = sha(data)
        if rel.endswith(OMIT_SUFFIX):
            manifest['omitted'].append(rel); continue
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
    (root / 'MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=1, sort_keys=True), encoding='utf-8')
    for dirpath, dirs, files in os.walk(root):
        for f in files:
            fp = Path(dirpath) / f
            fp.chmod(fp.stat().st_mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
    print(name, len(manifest['files']), 'files,', len(manifest['omitted']), 'omitted')

def zip_members(path):
    z = zipfile.ZipFile(path)
    out = []
    for info in z.infolist():
        name = info.filename
        if not (info.flag_bits & 0x800):
            try: name = name.encode('cp437').decode('utf-8')
            except UnicodeError: pass
        out.append((name, z.read(info)))
    return out

def main():
    uploads, fix5zip, out = map(Path, sys.argv[1:4])
    out.mkdir(parents=True, exist_ok=True)
    a = next(uploads.glob('365e0e0b-*.zip')); b = next(uploads.glob('c00568ae-*.zip'))
    for name, path in (('fix4', a), ('fix4_p1p5', b), ('fix5', fix5zip)):
        data = path.read_bytes(); members = zip_members(path)
        strip = members[0][0].split('/')[0] + '/'
        write_tree(name, members, out, path, sha(data), strip)
        if name == 'fix5':
            inner = dict(members)['linecar_optimized/docs/fix5/checkpoints/fix4_baseline.tar.gz']
            t = tarfile.open(fileobj=io.BytesIO(inner))
            mem = [(m.name, t.extractfile(m).read()) for m in t.getmembers() if m.isfile()]
            write_tree('opt', mem, out, Path('linecar_FIX5.zip!docs/fix5/checkpoints/fix4_baseline.tar.gz'),
                       sha(inner), mem[0][0].split('/')[0] + '/')

if __name__ == '__main__':
    main()
