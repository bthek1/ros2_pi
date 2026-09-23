#!/usr/bin/env bash
#
# Assert nothing this workspace starts is still running, on either machine.
# Prints a count per host and exits non-zero if any survived, which is what
# makes it an assertion rather than a utility — gates/teardown.sh uses it as
# its test, so it has to be runnable outside `just` and outside this box.

source "$(dirname "${BASH_SOURCE[0]}")/lib/just-lib.sh"

# The dev-box sweep is `pimesh_local_processes` in tools/lib/just-lib.sh, which drops
# the caller's own process group — `pgrep -f` reads command lines, and the command
# line asking the question is one of them, so a terminal command that merely
# *mentions* a pattern makes this script report itself. A genuine straggler is by
# definition something whose session has gone, which puts it in another group.
#
# The Pi half is `pimesh_pi_processes`, in the same file. Both live there rather
# than here because `assert_no_session` asks the same question before a session
# starts, and "what of ours is running" must not have two spellings — that is the
# drift tools/lib/just-lib.sh exists to prevent. Read the note on the Pi one: an
# unreachable Pi is reported as a clean Pi, which is this script's one soft spot.

total=0
for host in dev pi; do
    found=""
    if [[ $host == dev ]]; then
        hits=$(pimesh_local_processes)
        [[ -n $hits ]] && found+="${hits}"$'\n'
    else
        hits=$(pimesh_pi_processes)
        [[ -n $hits ]] && found+="${hits}"$'\n'
    fi
    count=$(grep -c . <<<"${found%$'\n'}" || true)
    [[ -z ${found//[$'\n' ]/} ]] && count=0
    echo "stragglers on ${host}: ${count}"
    # sed, not printf: `printf '  %s\n'` on a multi-line string indents only the
    # first line, so a Pi with three of ours on it printed one indented entry and
    # two flush-left ones that read like a new section. Same spelling as
    # assert_no_session, which carries the same note.
    [[ $count -gt 0 ]] && sed 's/^/  /' <<<"${found%$'\n'}"
    total=$(( total + count ))
done

if (( total > 0 )); then
    echo "FAIL: ${total} process(es) outlived their session"
    exit 1
fi
