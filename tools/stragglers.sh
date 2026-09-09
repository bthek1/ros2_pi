#!/usr/bin/env bash
#
# Assert nothing this workspace starts is still running, on either machine.
# Prints a count per host and exits non-zero if any survived, which is what
# makes it an assertion rather than a utility — gates/hello-clean.sh uses it as
# its test, so it has to be runnable outside `just` and outside this box.

source "$(dirname "${BASH_SOURCE[0]}")/just-lib.sh"

# `pgrep -f` reads command lines, and the command line asking the question is
# one of them — a terminal command that merely *mentions* a pattern makes this
# script report itself. Everything in the caller's own process group is the
# caller or its children, so drop that group. A genuine straggler is by
# definition something whose session has gone, which puts it in another one.
mypgid=$(ps -o pgid= -p $$ | tr -d ' ')
not_me() {
    while read -r pid rest; do
        [[ -z ${pid:-} ]] && continue
        pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)
        [[ $pgid == "$mypgid" ]] || echo "$pid $rest"
    done
}

total=0
for host in dev pi; do
    found=""
    for pat in "${PIMESH_PATTERNS[@]}"; do
        if [[ $host == dev ]]; then
            hits=$(pgrep -af "$pat" 2>/dev/null | not_me || true)
        else
            hits=$(pi_run "pgrep -af '$pat'" 2>/dev/null || true)
        fi
        [[ -n $hits ]] && found+="${hits}"$'\n'
    done
    count=$(grep -c . <<<"${found%$'\n'}" || true)
    [[ -z ${found//[$'\n' ]/} ]] && count=0
    echo "stragglers on ${host}: ${count}"
    [[ $count -gt 0 ]] && printf '  %s\n' "${found%$'\n'}"
    total=$(( total + count ))
done

if (( total > 0 )); then
    echo "FAIL: ${total} process(es) outlived their session"
    exit 1
fi
