#!/usr/bin/env bash
#
# Test gate: the unit tests pass on both machines, and there are some.
#
# Two assertions, and the second is the one that makes this a gate rather than a
# wrapper. `colcon test` exits 0 when a test *fails* — it is reporting that the
# test run completed — and it also exits 0 when a package contains no tests at
# all, which is exactly what an unbuilt tree looks like. So a naive
# `colcon test && echo PASS` is green under three different conditions, only one
# of which is "the tests pass". This asserts on the *counts*: zero failures, and
# a floor on the number of tests that actually ran.
#
# The floor is not a target. It is there so that deleting a test file, or a
# BUILD_TESTING guard quietly turning off, fails loudly instead of making the
# gate faster. Raise it when tests are added; never lower it to make a run pass.
#
# Both machines, because they compile different code from the same sources —
# x86_64 under Lyrical here, aarch64 under Jazzy there — and because a test
# suite that only runs on one machine is one that stops being run on the other.

source "$(dirname "${BASH_SOURCE[0]}")/../just-lib.sh"
echo "== gate-test =="

MIN_TESTS=29

fail=0
note() { echo "FAIL: $*"; fail=1; }

work=$(mktemp -d)

# `colcon test-result` prints a final line of the form
#   Summary: 29 tests, 0 errors, 0 failures, 0 skipped
# and that line is the evidence. Parsed rather than eyeballed, from each
# machine's own run.
summarise() {           # $1 = a log containing colcon test-result output
    grep -E '^Summary: [0-9]+ tests' "$1" | tail -n1
}
field() {               # $1 = summary line, $2 = word after the number
    awk -v key="$2" '{for (i = 1; i < NF; i++) if ($(i+1) ~ "^" key) {gsub(/[^0-9]/, "", $i); print $i; exit}}' <<<"$1"
}

for host in dev pi; do
    if [[ $host == dev ]]; then
        bash "$PIMESH_WS/tools/test.sh" >"$work/$host" 2>&1 || true
    else
        bash "$PIMESH_WS/tools/test-pi.sh" >"$work/$host" 2>&1 || true
    fi

    summary=$(summarise "$work/$host")
    if [[ -z ${summary:-} ]]; then
        note "no test summary from ${host} — the run did not get as far as testing"
        tail -20 "$work/$host" | sed 's/^/  /'
        continue
    fi

    tests=$(field "$summary" tests)
    errors=$(field "$summary" errors)
    failures=$(field "$summary" failures)
    skipped=$(field "$summary" skipped)

    printf '%-4s %s\n' "$host" "$summary"

    (( failures == 0 )) || note "${host}: ${failures} test failure(s)"
    (( errors == 0 )) || note "${host}: ${errors} test error(s)"
    (( tests >= MIN_TESTS )) ||
        note "${host}: only ${tests} tests ran, floor is ${MIN_TESTS} — did a suite stop being built?"
    (( skipped == 0 )) ||
        note "${host}: ${skipped} test(s) skipped — a skipped test asserts nothing"

    # Which suites ran, not just how many tests. A count can stay put while one
    # file's worth of tests is replaced by another's.
    grep -oE '[a-z0-9_]+\.(gtest|xunit)\.xml' "$work/$host" | sed 's/\..*//' | sort -u \
        >"$work/suites.$host"
    printf '     suites: %s\n' "$(tr '\n' ' ' <"$work/suites.$host")"

    eval "${host}_tests=\$tests"
done

# The same suites at both ends. A test that compiles here and not there is the
# ABI split showing up in the place it is cheapest to find it.
if [[ -s $work/suites.dev && -s $work/suites.pi ]]; then
    if ! diff -q "$work/suites.dev" "$work/suites.pi" >/dev/null; then
        note "the two machines ran different test suites"
        diff -u "$work/suites.dev" "$work/suites.pi" | sed 's/^/  /'
    fi
fi

echo
echo "tests            : dev=${dev_tests:-none}  pi=${pi_tests:-none}  (assert >= ${MIN_TESTS} on both)"
echo "failures/errors  : 0 required on both"
echo "suites           : $(tr '\n' ' ' <"$work/suites.dev" 2>/dev/null)  (assert identical on both)"

(( fail == 0 )) || { echo "FAIL gate-test"; exit 1; }
echo "PASS gate-test"
