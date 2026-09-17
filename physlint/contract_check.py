#!/usr/bin/env python3
"""Does the action-to-command mapping depend on a randomized parameter the policy can't see?

Static check over C sources with libclang. For every function that reads actions[...], taint the locals
derived from it, walk the tainted statements, and collect every struct field those statements depend on,
through locals and through callees. Intersect with the fields that are assigned from a random source at
reset and not written into the observation buffer. Report the intersection per statement with file:line.

A tainted statement whose left-hand side looks like state or a derivative is classified PLANT, a bare call
into a stepper is CALL, everything else is MAPPING. Only MAPPING is flagged. The classification is by name
and is printed so you can check it.

Usage: contract_check.py <dir> [--entry file.h] [-I dir] [-Dmacro=value] [--verbose]
"""
import sys, os, re, argparse
import clang.cindex as ci
from clang.cindex import CursorKind as K

RAND_NAMES = re.compile(r'(rand|rnd|random|uniform|noise|jitter|sample)', re.I)
PLANT_LHS = re.compile(r'(dot|accel|acc\b|force|torque|tau|vel\b|omega|state->|->state|d->)', re.I)

def walk(c):
    yield c
    for ch in c.get_children():
        yield from walk(ch)

def loc(c):
    f = c.location.file
    return f"{os.path.basename(f.name) if f else '?'}:{c.location.line}"

def src_of(c):
    ext = c.extent
    try:
        with open(ext.start.file.name, 'rb') as fh:
            data = fh.read()
        return data[ext.start.offset:ext.end.offset].decode('utf-8', 'replace')
    except Exception:
        return c.spelling

def in_target(c, root):
    f = c.location.file
    return bool(f) and os.path.abspath(f.name).startswith(root)

def collect_functions(tu, root):
    fns = {}
    for c in walk(tu.cursor):
        if c.kind == K.FUNCTION_DECL and c.is_definition() and in_target(c, root):
            fns[c.spelling] = c
    return fns

def qual(ch):
    """Struct.field for a MEMBER_REF_EXPR, using the field decl's parent type."""
    ref = ch.referenced
    parent = ref.semantic_parent if ref is not None else None
    sname = ''
    if parent is not None:
        sname = parent.spelling or parent.type.spelling or ''
        if not sname:
            # anonymous struct behind a typedef: walk up to the typedef
            try:
                sname = parent.type.get_canonical().spelling
            except Exception:
                sname = ''
    sname = sname.replace('struct ', '').strip() or '?'
    return f"{sname}.{ch.spelling}"

def fields_read(c):
    """qualified struct fields referenced under cursor c."""
    return {qual(ch) for ch in walk(c) if ch.kind == K.MEMBER_REF_EXPR and ch.spelling}

def callees(c):
    return {ch.spelling for ch in walk(c) if ch.kind == K.CALL_EXPR and ch.spelling}

def randomized_fields(fns):
    """fields assigned from an expression containing a random-looking call."""
    out = {}
    for name, fn in fns.items():
        for c in walk(fn):
            if c.kind == K.BINARY_OPERATOR and src_of(c).count('=') >= 1:
                kids = list(c.get_children())
                if len(kids) != 2: continue
                lhs, rhs = kids
                toks = [t.spelling for t in c.get_tokens()]
                # find top-level '=' (not '==', '<=', etc.)
                if '=' not in toks: continue
                lhs_fields = [qual(ch) for ch in walk(lhs) if ch.kind == K.MEMBER_REF_EXPR]
                if not lhs_fields: continue
                if any(RAND_NAMES.search(n) for n in callees(rhs)):
                    out.setdefault(lhs_fields[0], []).append((name, loc(c)))
    return out

def propagate_copies(fns, rand):
    """X.f = <expr reading Y.g> with Y.g randomized => X.f randomized (via copy). Iterate to a fixpoint."""
    assigns = []
    for name, fn in fns.items():
        for c in walk(fn):
            if c.kind == K.BINARY_OPERATOR:
                toks = [t.spelling for t in c.get_tokens()]
                if '=' not in toks: continue
                kids = list(c.get_children())
                if len(kids) != 2: continue
                lf = [qual(ch) for ch in walk(kids[0]) if ch.kind == K.MEMBER_REF_EXPR]
                if not lf: continue
                assigns.append((lf[0], fields_read(kids[1]), name, loc(c)))
    changed = True
    while changed:
        changed = False
        for lhs, rfs, name, where in assigns:
            if lhs not in rand and rfs and rfs <= set(rand):
                rand[lhs] = [(name, where + ' (copy of ' + ','.join(sorted(rfs & set(rand))) + ')')]
                changed = True
    return rand

OBS_NAMES = re.compile(r'^(observations?|obs)$')
def observed_fields(fns):
    """fields read by any function that writes into observations (plain or member ref), and its callees."""
    out = set()
    for name, fn in fns.items():
        writes = False
        for c in walk(fn):
            if c.kind in (K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR):
                kids = list(c.get_children())
                if kids and any(ch.kind in (K.DECL_REF_EXPR, K.MEMBER_REF_EXPR) and OBS_NAMES.match(ch.spelling or '') for ch in walk(kids[0])):
                    writes = True; break
        if writes:
            out |= fn_field_closure(name, fns)
    return out

def fn_field_closure(fname, fns, seen=None):
    """fields read by fname and everything it calls (transitively, within target)."""
    return set(fn_field_prov(fname, fns, seen))

def fn_field_prov(fname, fns, seen=None):
    """field -> name of the function where it is read, following calls transitively."""
    seen = seen if seen is not None else set()
    if fname in seen or fname not in fns: return {}
    seen.add(fname)
    fn = fns[fname]
    out = {f: fname for f in fields_read(fn)}
    for cal in callees(fn):
        for f, where in fn_field_prov(cal, fns, seen).items():
            out.setdefault(f, where)
    return out

def statements(fn):
    body = next((ch for ch in fn.get_children() if ch.kind == K.COMPOUND_STMT), None)
    if body is None: return []
    out = []
    def rec(c):
        for ch in c.get_children():
            if ch.kind in (K.DECL_STMT, K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR, K.CALL_EXPR, K.RETURN_STMT):
                out.append(ch)
            elif ch.kind in (K.COMPOUND_STMT, K.FOR_STMT, K.IF_STMT, K.WHILE_STMT, K.DO_STMT):
                rec(ch)
    rec(body)
    return out

def refs(c):
    return {ch.spelling for ch in walk(c) if ch.kind == K.DECL_REF_EXPR}

def lhs_base(c):
    """base variable of an lvalue: a[i].f -> a, p->f -> p, *p -> p"""
    while True:
        kids = list(c.get_children())
        if c.kind == K.DECL_REF_EXPR: return c.spelling
        if c.kind in (K.ARRAY_SUBSCRIPT_EXPR, K.MEMBER_REF_EXPR, K.UNARY_OPERATOR, K.PAREN_EXPR, K.UNEXPOSED_EXPR) and kids:
            c = kids[0]; continue
        return None

def defined_names(stmt):
    if stmt.kind == K.DECL_STMT:
        return {ch.spelling for ch in stmt.get_children() if ch.kind == K.VAR_DECL}
    if stmt.kind in (K.BINARY_OPERATOR, K.COMPOUND_ASSIGNMENT_OPERATOR):
        toks = [t.spelling for t in stmt.get_tokens()]
        if not any(t in ('=', '+=', '-=', '*=', '/=') for t in toks): return set()
        kids = list(stmt.get_children())
        if kids:
            b = lhs_base(kids[0])
            return {b} if b else set()
    return set()

def analyze(fns, rand):
    findings = []
    action_fns = {n for n, f in fns.items() if any('actions' in refs(s) for s in statements(f))}
    for name, fn in fns.items():
        stmts = statements(fn)
        if name not in action_fns: continue
        tainted = {'actions'}
        # forward taint over locals
        changed = True
        while changed:
            changed = False
            for s in stmts:
                if refs(s) & tainted:
                    new = defined_names(s) - tainted
                    if new: tainted |= new; changed = True
        # backward slice: defs of locals used by tainted statements
        defs = {}
        for s in stmts:
            for d in defined_names(s): defs.setdefault(d, []).append(s)
        for s in stmts:
            if not (refs(s) & tainted): continue
            if s.kind == K.CALL_EXPR and s.spelling in action_fns: continue   # pass-through, analyzed on its own
            if any(ch.kind == K.CALL_EXPR and ch.spelling in action_fns for ch in walk(s)): continue
            # direct fields + callee closure + fields from defs of locals it uses
            prov = {f: name for f in fields_read(s)}
            for cal in callees(s):
                for f, w in fn_field_prov(cal, fns).items(): prov.setdefault(f, w)
            frontier = list(refs(s) - {'actions'}); seen = set()
            while frontier:
                v = frontier.pop()
                if v in seen: continue
                seen.add(v)
                for ds in defs.get(v, []):
                    if ds is s: continue
                    for f in fields_read(ds): prov.setdefault(f, name)
                    for cal in callees(ds):
                        for f, w in fn_field_prov(cal, fns).items(): prov.setdefault(f, w)
                    frontier.extend(refs(ds) - seen)
            fields = set(prov)
            hit = sorted(fields & set(rand))
            if not hit: continue
            text = ' '.join(src_of(s).split())
            lhs = text.split('=')[0]
            if s.kind == K.CALL_EXPR: cls = 'CALL'
            else: cls = 'PLANT' if PLANT_LHS.search(lhs) else 'MAPPING'
            findings.append((cls, name, loc(s), text[:110], [f"{h} (read in {prov[h]})" for h in hit]))
    return findings

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--entry', action='append', default=[], help='file(s) to parse; default: every .c/.h in dir')
    ap.add_argument('-I', action='append', default=[])
    ap.add_argument('-D', action='append', default=[], help='macro definitions, e.g. -D__device__=')
    ap.add_argument('--verbose', action='store_true')
    a = ap.parse_args()
    root = os.path.abspath(a.dir)
    files = a.entry or sorted(os.path.join(root, f) for f in os.listdir(root) if f.endswith(('.c', '.h')))
    idx = ci.Index.create()
    fns = {}
    for f in files:
        args = ['-x', 'c', '-std=c11', f'-I{root}'] + [f'-I{i}' for i in a.I] + [f'-D{d}' for d in a.D]
        tu = idx.parse(f, args=args, options=ci.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES * 0)
        fns.update(collect_functions(tu, root))
    rand = propagate_copies(fns, randomized_fields(fns))
    obs = observed_fields(fns)
    # a field copied only from observed fields is observed too
    changed = True
    while changed:
        changed = False
        for k, v in rand.items():
            if k in obs: continue
            m = re.search(r'\(copy of ([^)]*)\)', v[0][1])
            if m and all(src in obs for src in m.group(1).split(',')):
                obs.add(k); changed = True
    hidden = {k: v for k, v in rand.items() if k not in obs}
    print(f"target: {root}\nfunctions parsed: {len(fns)}")
    print(f"randomized fields ({len(rand)}), of which observed by the policy: {len(rand)-len(hidden)}, hidden: {len(hidden)}")
    print("hidden randomized: " + (', '.join(f"{k} [{v[0][0]} {v[0][1]}]" for k, v in sorted(hidden.items())) or 'none'))
    findings = analyze(fns, hidden)
    flagged = [f for f in findings if f[0] == 'MAPPING']
    n = {c: sum(1 for f in findings if f[0] == c) for c in ('MAPPING', 'PLANT', 'CALL')}
    print(f"\ntainted statements depending on hidden randomized fields: MAPPING {n['MAPPING']} (flagged), PLANT {n['PLANT']}, CALL {n['CALL']}\n")
    for cls, fn, where, text, hit in findings:
        if cls != 'MAPPING' and not a.verbose: continue
        mark = 'FLAG' if cls == 'MAPPING' else '    '
        print(f"{mark} {cls:7} {where:24} in {fn}()\n       {text}\n       depends on hidden randomized: {', '.join(hit)}")
    if not findings: print("no action-path dependence on randomized fields found")
    return 1 if flagged else 0

if __name__ == '__main__':
    sys.exit(main())
