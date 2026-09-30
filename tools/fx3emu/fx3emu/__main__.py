"""fx3emu command line.

    python -m fx3emu diff [--a IMAGE] [--b IMAGE] [--only NAME,...] [--fuzz N]
    python -m fx3emu coverage [--image IMAGE] [--fuzz N]
    python -m fx3emu trace SCENARIO [--image IMAGE] [--grep TEXT]

IMAGE is `vendor` (EXStar's package, named via reference/vendor-names.elf), `build` (the open build's
output, default for --b) or a path to an ELF.
"""
import argparse, os, sys

from . import paths, scenarios
from .diff import differential, run, fmt
from .image import Image, library_names


def load(spec):
    if spec == "vendor":
        return Image.from_vendor(paths.VENDOR_PACKAGE, paths.VENDOR_NAMES_ELF, label="vendor")
    if spec == "build":
        return Image.from_elf(paths.MODERN_ELF, label="build")
    return Image.from_elf(spec)


def pick(args):
    all_ = scenarios.all_scenarios(range(args.fuzz))
    if args.only:
        want = args.only.split(",")
        all_ = [s for s in all_ if any(w in s.__name__ for w in want)]
    return all_


def app_functions(image):
    lib = library_names()
    return [s for s in image.funcs if s.name not in lib and not s.name.startswith("_") and s.size]


def coverage(image, devices, out=print):
    covered = set()
    for d in devices:
        for a, n in d.m.covered:
            covered.update(range(a, a + n, 4))
    tot_c = tot_w = 0
    rows = []
    for s in app_functions(image):
        words = image.code_words(s)
        c = sum(1 for w in words if w in covered)
        tot_c += c
        tot_w += len(words)
        rows.append((s.name, c, len(words)))
    for name, c, w in sorted(rows, key=lambda r: r[1] / max(r[2], 1)):
        out("  %-32s %5d / %5d  %5.1f%%" % (name, c, w, 100.0 * c / max(w, 1)))
    out("application code covered: %d / %d instructions (%.1f%%)" % (tot_c, tot_w, 100.0 * tot_c / max(tot_w, 1)))


def stacks(devices, out=print):
    worst = {}
    for d in devices:
        for name, size, used in d.m.stack_report():
            if used is not None and used > worst.get(name, (size, -1))[1]:
                worst[name] = (size, used)
    for name, (size, used) in sorted(worst.items()):
        out("  %-28s %s bytes used%s" % (name, used, " of %d" % size if size else " (callback stack)"))


def main(argv=None):
    ap = argparse.ArgumentParser(prog="fx3emu")
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("diff")
    p.add_argument("--a", default="vendor")
    p.add_argument("--b", default="build")
    p.add_argument("--only")
    p.add_argument("--fuzz", type=int, default=8)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--strict", action="store_true", help="show every diff, ignoring expected.py")
    p.add_argument("--no-faults", action="store_true", help="disable I2C fault injection (strong equivalence gate)")
    p = sub.add_parser("coverage")
    p.add_argument("--image", default="build")
    p.add_argument("--only")
    p.add_argument("--fuzz", type=int, default=8)
    p = sub.add_parser("trace")
    p.add_argument("scenario")
    p.add_argument("--image", default="build")
    p.add_argument("--grep")
    args = ap.parse_args(argv)

    if args.cmd == "diff":
        a, b = load(args.a), load(args.b)
        print("A = %s, B = %s" % (a.label, b.label))
        results, devices = differential(a, b, pick(args), seed=args.seed, strict=args.strict, faults=not args.no_faults)
        bad = [k for k, v in results.items() if v]
        print("\nstack use (B):")
        stacks(devices)
        print("\n%d scenarios, %d differ%s" % (len(results), len(bad), (": " + ", ".join(bad)) if bad else ""))
        return 1 if bad else 0
    if args.cmd == "coverage":
        img = load(args.image)
        devices = [run(img, s) for s in pick(args)]
        coverage(img, devices)
        print("\nstack use:")
        stacks(devices)
        return 0
    if args.cmd == "trace":
        img = load(args.image)
        sc = [s for s in scenarios.all_scenarios(range(64)) if s.__name__ == args.scenario]
        if not sc:
            sys.exit("no scenario %s" % args.scenario)
        d = run(img, sc[0])
        for ev in d.trace:
            line = fmt(ev)
            if not args.grep or args.grep in line:
                print(line)
        return 0


if __name__ == "__main__":
    sys.exit(main())
