#!/usr/bin/env bash
#
# Justfile gate (gh issue #3): the justfile is the user-facing surface of the
# workspace, every recipe is filed under a group, and the bash it used to inline
# is in tools/ where a linter can finally see it.
#
# The properties this asserts are the ones that decay silently. A recipe added
# without a group does not break anything, it just makes `just --list` slightly
# less useful, and then so does the next one; bash written inline is invisible to
# every linter; and a body that grows past a screen is how the file got to 627
# lines the first time.
#
# WANT_GROUPS is the sharp one now. The file was trimmed to `build` and `run`
# because seven gate recipes had buried `hello-compose`, the command a person
# new to the workspace actually wants; gates are run as `bash tools/gates/*.sh`.
# Asserting the group list is what stops them drifting back in one at a time —
# the failure mode is not a bad recipe, it is a good recipe in a file that is no
# longer a short answer to "what do I type?".

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh"
echo "== gate-justfile =="

MAX_LINES=80
MAX_BODY=10
WANT_GROUPS="build run"

fail=0
note() { echo "FAIL: $*"; fail=1; }

# 1. Groups. `just --dump --dump-format json` is the authority rather than the
#    rendered `--list`, because it reports the attributes as parsed: a group
#    that is a typo shows up here as a fifth group name rather than as a heading
#    a reader has to notice.
dump=$(mktemp)
just --justfile "$PIMESH_WS/justfile" --dump --dump-format json >"$dump"

# `default` is exempt, and only `default`: it is the list itself, and filing
# the map inside one of the territories it describes would be worse than
# leaving it at the top.
read -r n_recipes n_ungrouped <<<"$(python3 - "$dump" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))["recipes"]
ungrouped = [n for n, r in d.items()
             if n != "default" and not any(
                 a == "group" or (isinstance(a, dict) and "group" in a)
                 for a in r["attributes"])]
print(len(d), len(ungrouped))
PY
)"
[[ $n_ungrouped -eq 0 ]] || {
    note "${n_ungrouped} of ${n_recipes} recipes carry no [group(...)]"
    python3 -c '
import json, sys
d = json.load(open(sys.argv[1]))["recipes"]
for n, r in sorted(d.items()):
    if n != "default" and not any(
            a == "group" or (isinstance(a, dict) and "group" in a) for a in r["attributes"]):
        print("  ungrouped:", n)
' "$dump"
}

groups_seen=$(just --justfile "$PIMESH_WS/justfile" --groups |
              awk 'NR > 1 && NF { print $1 }' | sort | tr '\n' ' ')
[[ ${groups_seen% } == "$WANT_GROUPS" ]] ||
    note "groups are [${groups_seen% }], expected [${WANT_GROUPS}]"

# 2. Size. The recipe body is the number that matters — a justfile can be short
#    and still hide a hundred lines of bash in one recipe.
lines=$(wc -l <"$PIMESH_WS/justfile")
(( lines < MAX_LINES )) || note "justfile is ${lines} lines, budget is < ${MAX_LINES}"

read -r longest_name longest_len <<<"$(python3 - "$dump" <<'PY'
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
#    asking), and the prelude. One copy each, in tools/just-lib.sh.
# The recipe *bodies*, from the dump, not the file: a comment explaining why the
# ssh options are what they are is the opposite of the problem, and grepping the
# raw file cannot tell the two apart.
bodies=$(mktemp)
python3 - "$dump" >"$bodies" <<'PY'
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
        note "recipe bodies still inline ${pat} (${hits}×) — it belongs in tools/just-lib.sh"
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
    diff -u "$pasted" "$listed" | head -20
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

echo
echo "recipes          : ${n_recipes}"
echo "ungrouped        : ${n_ungrouped} of ${n_recipes}  (assert 0; default exempt)"
echo "groups           : ${groups_seen% }  (assert ${WANT_GROUPS})"
echo "justfile lines   : ${lines}  (assert < ${MAX_LINES})"
echo "longest body     : ${longest_name}, ${longest_len} lines  (assert <= ${MAX_BODY})"
echo "inlined shell    : ${inlined}  (assert 0)"
echo "setup.md drift   : ${doc_diff} line(s) vs just --list  (assert 0)"
echo "shellcheck       : ${findings} finding(s) over $(find "$PIMESH_WS/tools" -name '*.sh' | wc -l) scripts  (assert 0)"

(( fail == 0 )) || { echo "FAIL gate-justfile"; exit 1; }
echo "PASS gate-justfile"
