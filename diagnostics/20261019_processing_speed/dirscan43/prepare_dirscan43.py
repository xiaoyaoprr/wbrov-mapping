"""Freeze a local-only inventory experiment; do not change candidate/runtime files."""
from pathlib import Path
import ast
import hashlib
import json

WORK = Path(__file__).resolve().parent
REPO = WORK.parents[2] / 'new-chat' / 'outputs' / 'wbrov-mapping'
STAGE = WORK / 'dirscan43'
source = (REPO / 'tools/deferred_mapping.py').read_text(encoding='utf-8')
expected = '562ad769b499e1da26beb258a57dbd80dae5b0bd64466f626d22173ff7b03605'
assert hashlib.sha256(source.encode()).hexdigest() == expected
old = '''    for p in paths:
        if p.is_symlink() or not (p.is_file() if spool else p.is_dir()) or p.parent != root:
            raise ValueError('missing or redirected recording segment: '+p.name)
'''
new = '''    # Fresh directory inventory on every call; never cache across seals or polls.
    # Only names/type checks are batched. Per-segment content hashing and native
    # before/after consumption checks remain unchanged.
    entries = {}
    if paths:
        expected_names = {p.name for p in paths}
        with os.scandir(root) as directory:
            for entry in directory:
                if entry.name in expected_names:
                    entries[entry.name] = entry
                    if len(entries) == len(expected_names):
                        break
    for p in paths:
        entry = entries.get(p.name)
        if (entry is None or entry.is_symlink() or
                not (entry.is_file() if spool else entry.is_dir()) or p.parent != root):
            raise ValueError('missing or redirected recording segment: '+p.name)
'''
assert source.count(old) == 1
candidate = source.replace(old, new)
base_ast, candidate_ast = ast.parse(source), ast.parse(candidate)
base_ast.body = [n for n in base_ast.body if not isinstance(n, ast.FunctionDef) or n.name != 'closed_inputs']
candidate_ast.body = [n for n in candidate_ast.body if not isinstance(n, ast.FunctionDef) or n.name != 'closed_inputs']
assert ast.dump(base_ast) == ast.dump(candidate_ast)
STAGE.mkdir()  # Exclusive: preserve each failed experiment.
(STAGE / 'baseline.py').write_text(source, encoding='utf-8', newline='\n')
(STAGE / 'candidate.py').write_text(candidate, encoding='utf-8', newline='\n')
manifest = {'experiment': 'dirscan43', 'scope': 'local Windows only; Jetson validation pending',
            'adopted': False, 'only_changed_function': 'closed_inputs',
            'baseline_sha256': expected, 'candidate_sha256': hashlib.sha256(candidate.encode()).hexdigest(),
            'repository_source_sha256': hashlib.sha256((REPO / 'tools/deferred_mapping.py').read_bytes()).hexdigest()}
(STAGE / 'source_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(json.dumps(manifest))
