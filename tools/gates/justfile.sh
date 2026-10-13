#!/usr/bin/env bash
#
# Justfile gate (gh issue #3): the justfile is the user-facing surface of the
# workspace, it lists one command, and the bash it used to inline is in tools/
# where a linter can finally see it.
#
# The properties this asserts are the ones that decay silently. A recipe added to
# the listing does not break anything, it just makes `just` slightly less of an
# answer, and then so does the next one; bash written inline is invisible to
# every linter; and a body that grows past a screen is how the file got to 627
# lines the first time.
#
# WANT_PUBLIC is the sharp one. The file has been cut twice — to `build` and
# `run` on 2026-09-09, when seven gate recipes had buried the command a newcomer
# wanted, and to `run` alone on 2026-10-13, when eight per-stage viewers had done
# the same to the SLAM pipeline. build, test and clean are [private]: they work
# and are not listed. Asserting the listing is what stops recipes drifting back
# in one at a time.

source "$(dirname "${BASH_SOURCE[0]}")/../lib/just-lib.sh"
echo "== gate-justfile =="

MAX_LINES=80
MAX_BODY=10
WANT_PUBLIC="run"

fail=0
note() { echo "FAIL: $*"; fail=1; }

# 1. The listing. `just --dump --dump-format json` is the authority rather than
#    the rendered `--list`, because it reports the attributes as parsed: a
#    `[private]` that is a typo shows up here as a recipe in the listing.
dump=$(mktemp)
just --justfile "$PIMESH_WS/justfile" --dump --dump-format json >"$dump"

read -r n_recipes public <<<"$(/usr/bin/python3 - "$dump" <<'PY2'
import json, sys
d = json.load(open(sys.argv[1]))["recipes"]
public = sorted(n for n, r in d.items()
                if not r.get("private") and "private" not in r["attributes"]
                and not n.startswith("_"))
print(len(d), ",".join(public))
PY2
)"
[[ ${public//,/ } == "$WANT_PUBLIC" ]] ||
    note "the listing is [${public//,/ }], expected [${WANT_PUBLIC}] — mark anything else [private]"

# 2. Size. The recipe body is the number that matters — a justfile can be short
#    and still hide a hundred lines of bash in one recipe.
lines=$(wc -l <"$PIMESH_WS/justfile")
(( lines < MAX_LINES )) || note "justfile is ${lines} lines, budget is < ${MAX_LINES}"

read -r longest_name longest_len <<<"$(/usr/bin/python3 - "$dump" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))["recipes"]
name, length = max(((n, len(r["body"])) for n, r in d.items()), key=lambda t: t[1])
print(name, length)
PY
)"
(( longest_len <= MAX_BODY )) ||
    note "recipe '${longest_name}' has ${longest_len} lines, budget is ${MAX_BODY}"

# 3. No inlined shell. Each of these is a thing that must be spelled the same
#    way everywhere: the SSH options (a bare ssh hangs two minutes on the Pi's
#    dead link), the bracketed kill patterns (the plain spelling kills the shell
#    asking), and the prelude. One copy each, in tools/lib/just-lib.sh.
# The recipe *bodies*, from the dump, not the file: a comment explaining why the
# ssh options are what they are is the opposite of the problem, and grepping the
# raw file cannot tell the two apart.
bodies=$(mktemp)
/usr/bin/python3 - "$dump" >"$bodies" <<'PY'
import json, sys


def text(line):
    return "".join(f if isinstance(f, str) else "{{...}}" for f in line)


d = json.load(open(sys.argv[1]))["recipes"]
for name, r in sorted(d.items()):
    for line in r["body"]:
        print(f"{name}: {text(line)}")
PY

inlined=0
for pat in 'set -euo pipefail' '\bssh ' '\bpkill\b' '\bpgrep\b' 'BatchMode'; do
    hits=$(grep -cE "$pat" "$bodies" || true)
    if (( hits > 0 )); then
        note "recipe bodies still inline ${pat} (${hits}×) — it belongs in tools/lib/just-lib.sh"
        grep -nE "$pat" "$bodies" | sed 's/^/  /'
        inlined=$(( inlined + hits ))
    fi
done

# 4. The docs quote the real list. A recipe list pasted into a doc is stale the
#    day after it is pasted, so this asserts the block in setup.md is what
#    `just --list` prints today rather than what it printed once.
doc="$PIMESH_WS/docs/info/setup.md"
listed=$(mktemp); pasted=$(mktemp)
just --justfile "$PIMESH_WS/justfile" --list >"$listed"
awk '/^```text$/ { grab = 1; next } /^```$/ { grab = 0 } grab' "$doc" >"$pasted"
doc_diff=$(diff -u "$pasted" "$listed" | grep -c '^[-+][^-+]' || true)
(( doc_diff == 0 )) || {
    note "docs/info/setup.md quotes a recipe list that is ${doc_diff} line(s) out of date"
    # `|| true`: diff exits 1 whenever it finds a difference, which under
    # pipefail and `set -e` ended this gate right here — so a drifted list hid
    # every check below it, and their silence read as passing (found 2026-10-13).
    diff -u "$pasted" "$listed" | head -20 || true
}

# 5. Lint. This is the payoff a 627-line justfile could not have: shellcheck
#    does not parse {{ }} interpolations, so none of this bash had ever been
#    linted before it moved into tools/.
if ! command -v shellcheck >/dev/null 2>&1; then
    note "shellcheck is not installed — 'uv tool install shellcheck-py' (no sudo needed)"
    findings=unknown
else
    mapfile -t scripts < <(find "$PIMESH_WS/tools" -name '*.sh' | sort)
    sc=$(mktemp)
    shellcheck -S warning -x "${scripts[@]}" >"$sc" 2>&1 || true
    findings=$(grep -c '^In .* line' "$sc" || true)
    if (( findings > 0 )); then
        note "shellcheck reports ${findings} finding(s) at -S warning"
        head -40 "$sc"
    fi
fi

# --- Every default argument survives the trip into the script ----------------
#
# **This is the check that was missing on 2026-09-23, and `just view-odom`
# failed for a user because of it.** A recipe body writes
# `{{ seconds }} {{ bag }} {{ regime }}`; just substitutes the text and hands the
# line to sh, so a parameter defaulting to "" **vanishes from the word list**
# and every argument after it shifts one place left. `just view-odom` therefore
# ran `view-odom.sh 600 sixdof`, the script read `sixdof` as the bag name, and
# the session died with "no bag at 'sixdof'". `dashboard` had it too, one
# argument along: its port would have been read as the bag.
#
# **Nothing covered it, and the reason is worth keeping.** `gates/teardown.sh`
# exercised both recipes — and called the view-odom script *directly*, with a bag
# it names itself, because it needs the process group. So the one gate that
# starts these recipes never goes through the justfile, and the justfile is the
# user-facing surface. Ask what the gate does **not** touch.
#
# `just -n` is the instrument rather than a regex over the body, because what is
# being checked is *just's own substitution*, not our model of it. An empty
# argument that survives prints as "" and counts as a word; one that vanished
# does not.
missing_args=0
params_of() {           # $1 = recipe, $2 = optional kind; prints parameter names, one per line
    /usr/bin/python3 -c '
import json, sys
r = json.load(open(sys.argv[1]))["recipes"][sys.argv[2]]
kind = sys.argv[3] if len(sys.argv) > 3 else None
print("\n".join(p["name"] for p in r.get("parameters", []) if kind in (None, p.get("kind"))))
' "$dump" "$1" ${2:+"$2"}
}
while read -r name want variadic; do
    # **2>&1, and the first version of this check had 2>/dev/null.** `just -n`
    # echoes the command to *stderr*, so discarding stderr left $line empty, the
    # loop skipped every recipe, and the gate printed "0 recipe(s) lose an
    # argument" over the two that did. Verified against the broken justfile
    # before being kept — which is the only reason that was found.
    #
    # **`--no-deps`, or a recipe with a dependency is never checked.** `run`
    # depends on `build`, so `just -n run` prints build's line first and `head -1`
    # read that: with `bag:=` deleted from run's body this check said 0 recipes
    # lose an argument (2026-10-13, run against the mutation before being kept).
    line=$(just --justfile "$PIMESH_WS/justfile" -n --no-deps "$name" 2>&1 | head -1) || true
    # A recipe with a required positional refuses to dry-run; nothing to check.
    [[ $line == bash\ * ]] || continue
    # The body is `bash <script> <args...>`, shell-quoted by just.
    words=()
    eval "words=( $line )" 2>/dev/null || continue
    # **A session recipe is checked by name, not by count** (#15). Its body is
    # `bash tools/session.sh <label> --pi|--local -- ros2 launch … name:="{{ name }}"`,
    # and an empty value cannot vanish from `name:=` — the key holds its place.
    # What can go wrong instead is a parameter that never reaches the launch at
    # all, or one spelled differently from the launch argument it feeds, which
    # the launch then ignores while running on its default. So every parameter
    # has to appear as its own `name:=` word.
    #
    # A variadic parameter (`run`'s *launch_args) is the exception: it is a list
    # of `name:=value` words already, forwarded as they are.
    if [[ ${words[1]} == */tools/session.sh ]]; then
        for p in $(params_of "$name" singular); do
            printf '%s\n' "${words[@]}" | grep -q "^${p}:=" || {
                note "recipe '${name}' never passes parameter '${p}' to its launch as ${p}:="
                printf '  just -n %s -> %s\n' "$name" "$line"
                missing_args=$(( missing_args + 1 ))
            }
        done
        continue
    fi
    # A variadic parameter may legitimately expand to any number of words,
    # itself included: that is the whole point of `build *args`.
    (( variadic == 0 )) || continue
    got=$(( ${#words[@]} - 2 ))
    if (( got != want )); then
        note "recipe '${name}' passes ${got} argument(s) where it has ${want} parameter(s) — an empty default vanished; quote the {{ ... }} in its body"
        printf '  just -n %s -> %s\n' "$name" "$line"
        missing_args=$(( missing_args + 1 ))
    fi
done < <(/usr/bin/python3 -c '
import json, sys
recipes = json.load(open(sys.argv[1]))["recipes"]
for name, recipe in sorted(recipes.items()):
    params = recipe.get("parameters", [])
    if not params:
        continue
    variadic = int(any(p.get("kind") != "singular" for p in params))
    print(name, len(params), variadic)
' "$dump")

echo
echo "recipes          : ${n_recipes}"
echo "listed           : ${public//,/ } of ${n_recipes} recipes  (assert ${WANT_PUBLIC}; the rest [private])"
echo "justfile lines   : ${lines}  (assert < ${MAX_LINES})"
echo "longest body     : ${longest_name}, ${longest_len} lines  (assert <= ${MAX_BODY})"
echo "inlined shell    : ${inlined}  (assert 0)"
echo "argument drift   : ${missing_args} recipe(s) lose an argument  (assert 0; just -n)"
echo "setup.md drift   : ${doc_diff} line(s) vs just --list  (assert 0)"
echo "shellcheck       : ${findings} finding(s) over $(find "$PIMESH_WS/tools" -name '*.sh' | wc -l) scripts  (assert 0)"

(( fail == 0 )) || { echo "FAIL gate-justfile"; exit 1; }
echo "PASS gate-justfile"
