"""Inventory of application globals (g_XXXXXXXX) across the tree: declared types and use forms."""
import collections, os, re, sys
T = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
decl = collections.defaultdict(set); uses = collections.defaultdict(collections.Counter)
mods = collections.defaultdict(set)
for m in sorted(f for f in os.listdir(os.path.join(T, "src")) if f.endswith(".c")):
    s = open(os.path.join(T, "src", m)).read()
    for d in re.finditer(r"^extern\s+([^;]*?)\b(g_[0-9a-f]{8}\w*)\s*(\[[^\]]*\])?\s*;", s, re.M):
        decl[d.group(2)].add(re.sub(r"\s+", " ", d.group(1).strip() + (" []" if d.group(3) else "")))
    body = re.sub(r"^extern[^;]*;", "", s, flags=re.M)
    for u in re.finditer(r"(&\s*)?\b(g_[0-9a-f]{8}\w*)\b(\s*(?:\[|\.|->))?", body):
        kind = "addr" if u.group(1) else ("member" if u.group(3) else "value")
        uses[u.group(2)][kind] += 1; mods[u.group(2)].add(m)
for g in sorted(set(decl) | set(uses)):
    if len(sys.argv) > 1 and not any(g.startswith(p) for p in sys.argv[1:]): continue
    print("%-16s %-40s %-28s %s" % (g, " | ".join(sorted(decl[g])), dict(uses[g]), ",".join(sorted(mods[g]))))
