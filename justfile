# ros2_pi — day-to-day commands. `just` with no arguments lists them.
#
# Anything longer than one line is a shebang recipe, so that shell variables
# survive from line to line; `set shell` below only governs the one-liners.
#
# Recipes that touch ROS source tools/ros-env.sh instead of naming a distro.
# The dev box is Lyrical, the Pi is Jazzy, and this file is rsynced to the Pi
# (P3) — a hard-coded /opt/ros/lyrical would be wrong at the far end.
#
# The gates: every phase of gh issue #2 ends in one, and a gate exits non-zero
# and prints the number it asserted on. `just gate-hello-build` is P0's.

set shell := ["bash", "-euo", "pipefail", "-c"]

ws := justfile_directory()

# ament_cmake is not a pure-CMake buildtool: it shells out to Python at
# *configure* time (package_xml_2_cmake.py and friends), so a C++ package still
# needs an interpreter that can import catkin_pkg. CMake's FindPython3 picks the
# highest version it can see, and this machine has two 3.14s on PATH — the apt
# one in /usr/bin that ROS's dist-packages belong to, and a uv-managed one in
# ~/.local/bin that has never heard of catkin_pkg. Naming the interpreter is the
# only way to stop that coin-flip; the alternative (reordering PATH) leaves the
# choice to whichever Python sorts highest. Measured 2026-09-08: without this,
# every ament_cmake package fails at ament_package() with ModuleNotFoundError.
# /usr/bin/python3 is the right answer on both machines — both take ROS from apt.
cmake_args := "-DPython3_EXECUTABLE=/usr/bin/python3"

# List the recipes
default:
    @just --list --unsorted

# Build the workspace
build:
    #!/usr/bin/env bash
    set -euo pipefail
    source "{{ ws }}/tools/ros-env.sh"
    cd "{{ ws }}"
    colcon build --symlink-install --cmake-args {{ cmake_args }}

# Delete the colcon trees (they are git-ignored; nothing else notices)
clean:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    rm -rf build install log
    echo "removed build/ install/ log/"

# Assert nothing this workspace starts is still running, on either machine
stragglers:
    #!/usr/bin/env bash
    set -euo pipefail

    # Bracket forms throughout: `pgrep -f component_container` matches the shell
    # command that contains those characters, so the plain spelling reports
    # itself as a straggler and, with pkill, kills the session asking.
    # Path-qualified, and bracketed. Bracketed because `pgrep -f
    # <plain-word>` matches the shell command that contains those characters —
    # the plain spelling reports itself, and with pkill it kills the session
    # asking. Path-qualified because the bracket only protects the pattern's own
    # text: a command mentioning the bare word anywhere else still matches, so
    # these anchor on the installed binary paths, which prose does not contain.
    patterns=(
        '/lib/[p]imesh_[a-z]*/'
        'rclcpp_components/[c]omponent_container'
        'ros2 launch pimesh_[a-z]*'
    )

    # `pgrep -f` reads command lines, and the command line asking the question
    # is one of them — a terminal command that merely *mentions* a pattern makes
    # this recipe report itself. Everything in the caller's own process group is
    # the caller or its children, so drop that group. A genuine straggler is by
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
        for pat in "${patterns[@]}"; do
            if [[ $host == dev ]]; then
                hits=$(pgrep -af "$pat" 2>/dev/null | not_me || true)
            else
                hits=$(ssh {{ ssh_opts }} {{ pi }} "bash -lc 'pgrep -af \"$pat\"'" 2>/dev/null || true)
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

# P0 gate: one real package builds, and nothing stale pretends to be one
gate-hello-build:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    echo "== gate-hello-build =="

    # 1. Nothing stale.
    #
    # A colcon workspace is an *overlay*: install/setup.bash prepends prefixes
    # to AMENT_PREFIX_PATH, and nothing ever removes a package from that tree
    # when its source disappears. So `ros2 pkg list` is a statement about
    # install/, not about src/, until someone makes the two agree. That is what
    # this loop is: the only way to keep the package list honest.
    # The check itself lives in tools/check-stale.sh, because P3 runs the same
    # one on the Pi over SSH and one implementation is the point.
    if ! stale=$(bash tools/check-stale.sh); then
        echo "FAIL: build artefact(s) with no src/:"
        printf '  %s\n' $stale
        echo "  fix: just clean"
        exit 1
    fi
    echo "stale artefacts in build/ install/ log/latest_build: 0"

    # 2. Build, timed. Via the recipe, so the gate proves the recipe.
    start=$(date +%s.%N)
    {{ just_executable() }} build
    elapsed=$(awk -v a="$start" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')

    # 3. The package list says exactly what src/ says.
    source "{{ ws }}/tools/ros-env.sh" --overlay
    mapfile -t pimesh < <(ros2 pkg list | grep '^pimesh_' || true)
    if [[ ${#pimesh[@]} -ne 1 || ${pimesh[0]} != pimesh_hello ]]; then
        echo "FAIL: expected exactly [pimesh_hello], ros2 pkg list gave [${pimesh[*]-}]"
        exit 1
    fi

    echo
    echo "ROS_DISTRO      : ${ROS_DISTRO}"
    echo "build time      : ${elapsed}s"
    echo "pimesh packages : ${pimesh[*]}"
    echo "PASS gate-hello-build"

# P1 gate: the talker publishes at the rate it was asked for, and refuses a rate it was not
gate-hello-talk:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    source "{{ ws }}/tools/ros-env.sh" --overlay
    echo "== gate-hello-talk =="

    # Nothing this gate starts outlives it, including on the failure paths.
    # Pattern-match the node binary, never the `ros2 run` wrapper: killing the
    # wrapper orphans the binary, which is how a workspace ends up with a
    # publisher nobody can find on a topic somebody is still debugging.
    node_pat='pimesh_hello/lib/pimesh_hello/hello_node'
    cleanup() { pkill -f "$node_pat" 2>/dev/null || true; }
    trap cleanup EXIT INT TERM HUP

    start_node() {          # $1 = log path, rest = --ros-args ...
        local log=$1; shift
        timeout -s INT 30 ros2 run pimesh_hello hello_node "$@" >"$log" 2>&1 &
        for _ in $(seq 40); do
            if ros2 topic list 2>/dev/null | grep -qx /hello_node/hello; then return 0; fi
            sleep 0.25
        done
        echo "FAIL: /hello_node/hello never appeared within 10 s"; cat "$log"; return 1
    }

    # Sample for long enough that the --window 10 average is over ten real
    # intervals rather than the two the tool starts with, and read the *last*
    # line for the same reason.
    #
    # Through a file rather than a pipe, deliberately: `timeout ... | awk` puts
    # awk in the signal's blast radius, so the timeout kills the parser along
    # with the thing being parsed and END never runs. Measured — it reported
    # every rate as empty. Two commands and a temp file have no such coupling.
    measure_rate() {        # $1 = seconds to sample -> prints Hz
        local out; out=$(mktemp)
        timeout -s INT "$1" ros2 topic hz /hello_node/hello --window 10 >"$out" 2>/dev/null || true
        awk '/average rate/ { r = $3 } END { if (r == "") exit 1; print r }' "$out"
    }

    in_range() {            # $1 = value, $2 = low, $3 = high
        awk -v v="$1" -v lo="$2" -v hi="$3" 'BEGIN { exit !(v >= lo && v <= hi) }'
    }

    log=$(mktemp -d)/hello.log

    # 1. The default rate is the declared default.
    start_node "$log"
    rate_default=$(measure_rate 13)
    in_range "$rate_default" 0.9 1.1 ||
        { echo "FAIL: default rate ${rate_default} Hz outside 0.9–1.1"; exit 1; }
    cleanup; sleep 1

    # 2. The payload is the declared default, and it is exactly that.
    start_node "$log"
    rate_echo=$(measure_rate 13)
    echo_out=$(mktemp)
    timeout -s INT 10 ros2 topic echo --once --field data /hello_node/hello >"$echo_out" 2>/dev/null || true
    text=$(head -1 "$echo_out")
    [[ $text =~ ^hello\ world$ ]] ||
        { echo "FAIL: echoed \"${text}\", expected \"hello world\""; exit 1; }
    in_range "$rate_echo" 0.9 1.1 ||
        { echo "FAIL: rate ${rate_echo} Hz outside 0.9–1.1 on the echo run"; exit 1; }
    cleanup; sleep 1

    # 3. The parameter is wired to the timer, not decorative. A node that
    #    accepts rate_hz and ignores it passes every check but this one.
    start_node "$log" --ros-args -p rate_hz:=5.0
    rate_five=$(measure_rate 10)
    in_range "$rate_five" 4.5 5.5 ||
        { echo "FAIL: rate_hz:=5.0 gave ${rate_five} Hz, outside 4.5–5.5"; exit 1; }
    cleanup; sleep 1

    # 4. A value outside the descriptor's range is refused at construction.
    #    Clamping and carrying on would be the friendly thing to do and the
    #    wrong one: a parameter that silently means something else is the bug
    #    class this whole declaration style exists to prevent.
    bad_start=$(date +%s.%N)
    set +e
    timeout -s INT 10 ros2 run pimesh_hello hello_node --ros-args -p rate_hz:=0.0 >"$log" 2>&1
    bad_exit=$?
    set -e
    bad_secs=$(awk -v a="$bad_start" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')
    if (( bad_exit == 0 )); then
        echo "FAIL: rate_hz:=0.0 was accepted (exit 0)"; exit 1
    fi
    if awk -v s="$bad_secs" 'BEGIN { exit !(s > 2.0) }'; then
        echo "FAIL: rate_hz:=0.0 took ${bad_secs}s to fail, budget is 2.0s"; exit 1
    fi
    grep -q "floating point range" "$log" ||
        { echo "FAIL: exited non-zero but never named the range"; tail -3 "$log"; exit 1; }

    echo
    echo "rate @ default   : ${rate_default} Hz   (assert 0.9–1.1)"
    echo "rate @ echo run  : ${rate_echo} Hz   (assert 0.9–1.1)"
    echo "rate @ 5.0       : ${rate_five} Hz   (assert 4.5–5.5)"
    echo "text             : \"${text}\""
    echo "rate_hz:=0.0     : exit ${bad_exit} after ${bad_secs}s (assert non-zero, < 2.0s)"
    echo "PASS gate-hello-talk"

# P2 gate: two components, one process, and the message handed over as a pointer
gate-hello-ipc:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    source "{{ ws }}/tools/ros-env.sh" --overlay
    echo "== gate-hello-ipc =="

    # The bracket is not decoration. `pkill -f component_container_mt` typed at a
    # terminal matches the shell command that contains those very characters and
    # kills the shell running it — measured, the hard way. `[c]omponent…` matches
    # the process and not the pattern. Container first, launcher second: killing
    # the launcher first orphans the container.
    cleanup() {
        pkill -f '[c]omponent_container_mt' 2>/dev/null || true
        sleep 0.5
        pkill -f '[h]ello.launch.py' 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM HUP

    start_container() {     # $1 = log path, $2 = true|false
        timeout -s INT 40 ros2 launch pimesh_hello hello.launch.py \
            intra_process:="$2" >"$1" 2>&1 &
        for _ in $(seq 60); do
            if ros2 node list 2>/dev/null | grep -qx /echo_node; then return 0; fi
            sleep 0.25
        done
        echo "FAIL: /echo_node never appeared within 15 s"; cat "$1"; return 1
    }

    # How many of the addresses the subscriber logged were addresses the
    # publisher logged. Under intra-process this is every one of them, because
    # it is the same object; under serialisation it is whatever malloc happens
    # to hand back, which is not zero — hence the control run below.
    match_rate() {          # $1 = log -> prints "<hits> <total>"
        awk '
            /pub seq=/ {
                if (match($0, /payload=0x[0-9a-f]+/))
                    pub[substr($0, RSTART + 8, RLENGTH - 8)] = 1
            }
            /echo n=/ {
                if (match($0, /payload=0x[0-9a-f]+/)) {
                    a = substr($0, RSTART + 8, RLENGTH - 8)
                    total++
                    if (a in pub) hits++
                }
            }
            # The trailing newline matters: `read` returns 1 at EOF without
            # one, and under `set -e` that ends the gate with no message at all.
            END { printf "%d %d\n", hits + 0, total + 0 }
        ' "$1"
    }

    tmp=$(mktemp -d)

    # ---- Run 1: intra-process on, the claim ------------------------------
    start_container "$tmp/on.log" true

    # Both components, and exactly one process holding them. A container per
    # component would pass every other assertion in this gate and none of the
    # ones the architecture actually needs.
    nodes=$(ros2 node list 2>/dev/null | sort | tr '\n' ' ')
    for n in /hello_node /echo_node; do
        grep -q -- "$n" <<<"$nodes" || { echo "FAIL: $n missing from: $nodes"; exit 1; }
    done
    containers=$(pgrep -fc '[c]omponent_container_mt' || echo 0)
    if [[ $containers -ne 1 ]]; then
        echo "FAIL: expected 1 component_container_mt process, found ${containers}"; exit 1
    fi
    pid=$(pgrep -f '[c]omponent_container_mt' | head -1)

    # The YAML applied, not merely loaded. `rate_hz` is 2.0 in config/hello.yaml
    # and 1.0 in the code; asking the running node which one it has is the only
    # question whose answer distinguishes a keyed file from a mis-keyed one.
    param_out=$(ros2 param get /hello_node rate_hz 2>&1)
    rate=$(awk '{ print $NF }' <<<"$param_out")
    awk -v v="$rate" 'BEGIN { exit !(v > 1.99 && v < 2.01) }' ||
        { echo "FAIL: rate_hz is ${rate}, expected 2.0 from config/hello.yaml"; exit 1; }

    sleep 8
    cleanup; sleep 1
    read -r on_hits on_total < <(match_rate "$tmp/on.log")
    if [[ $on_total -lt 5 ]]; then
        echo "FAIL: only ${on_total} messages received, need ≥ 5 to say anything"; exit 1
    fi
    if [[ $on_hits -ne $on_total ]]; then
        echo "FAIL: intra-process on, only ${on_hits}/${on_total} addresses matched"; exit 1
    fi
    on_pub=$(grep -m1 -o 'pub seq=1 payload=0x[0-9a-f]*' "$tmp/on.log" | grep -o '0x.*')
    on_echo=$(grep -m1 -o 'echo n=1 .*payload=0x[0-9a-f]*' "$tmp/on.log" | grep -o '0x.*')

    # ---- Run 2: intra-process off, the control ---------------------------
    #
    # Without this the gate proves nothing. Two allocations in one process can
    # share an address by coincidence — the publisher frees, the subscriber
    # allocates the same size, and malloc obliges. So the claim is not "the
    # addresses matched", it is "the addresses matched *and stop matching the
    # moment the mechanism is switched off*". Same binary, same launch file,
    # one flag.
    start_container "$tmp/off.log" false
    sleep 8
    cleanup; sleep 1
    read -r off_hits off_total < <(match_rate "$tmp/off.log")
    if [[ $off_total -lt 5 ]]; then
        echo "FAIL: control run received ${off_total} messages, need ≥ 5"; exit 1
    fi
    off_pct=$(( 100 * off_hits / off_total ))
    if [[ $off_pct -gt 25 ]]; then
        echo "FAIL: control matched ${off_hits}/${off_total} (${off_pct}%) — at that"
        echo "      rate address equality is coincidence, and run 1 proves nothing"
        exit 1
    fi

    echo
    echo "nodes             : ${nodes}"
    echo "container pid     : ${pid}  (assert exactly 1 process)"
    echo "rate_hz           : ${rate}  (assert 2.0, the YAML value, not 1.0)"
    echo "first pub payload : ${on_pub}"
    echo "first echo payload: ${on_echo}"
    echo "intra-process ON  : ${on_hits}/${on_total} addresses matched (assert all)"
    echo "intra-process OFF : ${off_hits}/${off_total} = ${off_pct}% (assert ≤ 25%)"
    echo "PASS gate-hello-ipc"

# --- The Pi -----------------------------------------------------------------
#
# Every ssh here carries `-o BatchMode=yes -o ConnectTimeout=5`. The Pi's Wi-Fi
# link dies while the Pi keeps running, and a bare ssh hangs about two minutes
# against a dead link — long enough to wedge whatever trap it is sitting in.
# `bash -lc` is equally non-negotiable: ROS_DOMAIN_ID and RMW_IMPLEMENTATION
# live in the Pi's ~/.profile, so a non-login shell runs on domain 0 with the
# wrong middleware and produces a result that means nothing.

pi := "pi"
pi_ws := "~/ros2_pi"
ssh_opts := "-o BatchMode=yes -o ConnectTimeout=5"

# Delete the Pi's colcon trees
clean-pi:
    #!/usr/bin/env bash
    set -euo pipefail
    ssh {{ ssh_opts }} {{ pi }} "bash -lc 'cd {{ pi_ws }} && rm -rf build install log'"
    echo "removed {{ pi_ws }}/{{ '{build,install,log}' }} on {{ pi }}"

# Ship source to the Pi. Source only — never a built tree
sync-pi:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"

    # src, tools and the justfile. Not build/, not install/, not log/ — and this
    # is the one thing in the project that cannot be worked around. ROS 2 makes
    # no ABI promise across distros, so a Lyrical .so does not load under Jazzy.
    # The Pi compiles the same source; it never receives a binary.
    rsync -a --delete -e "ssh {{ ssh_opts }}" \
        --exclude '__pycache__' \
        src tools justfile {{ pi }}:{{ pi_ws }}/

    # The sync may have just invalidated the Pi's build tree — `--delete` can
    # remove a package's sources while its artefacts stay behind, and a colcon
    # overlay never forgets a package on its own. Derived data, so clearing it
    # is safe; saying so is not optional.
    if ! stale=$(ssh {{ ssh_opts }} {{ pi }} "bash -lc 'cd {{ pi_ws }} && bash tools/check-stale.sh'"); then
        echo "sync-pi: the Pi's build tree has artefacts with no source:"
        printf '  %s\n' $stale
        echo "sync-pi: clearing it — colcon will not do this for you"
        just clean-pi
    fi
    echo "sync-pi: source is current on {{ pi }}"

# Build the workspace on the Pi, from source, under Jazzy
build-pi: sync-pi
    #!/usr/bin/env bash
    set -euo pipefail
    ssh {{ ssh_opts }} {{ pi }} "bash -lc 'cd {{ pi_ws }} && source tools/ros-env.sh && colcon build --symlink-install --cmake-args {{ cmake_args }}'"

# Talker on the Pi, listener here. seconds = how long the talker runs
hello-lan seconds="20":
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    source "{{ ws }}/tools/ros-env.sh" --overlay

    node_pat='[p]imesh_hello/lib/pimesh_hello'
    # Bash fires EXIT on Ctrl-C too, so one trap covers the interrupt, the
    # normal end and the error paths. It has to reach across the LAN: a talker
    # left running on the Pi is invisible from here and poisons the next run.
    cleanup() {
        pkill -f "$node_pat" 2>/dev/null || true
        ssh {{ ssh_opts }} {{ pi }} "bash -lc 'pkill -f \"$node_pat\" || true'" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM HUP

    marker="hello-from-$(ssh {{ ssh_opts }} {{ pi }} hostname)"
    echo "listener here (${ROS_DISTRO}), talker on {{ pi }}, payload ${marker}"

    # Listener first, so it is subscribed before the first message exists.
    timeout -s INT $(( {{ seconds }} + 15 )) ros2 run pimesh_hello echo_node &
    sleep 2
    ssh {{ ssh_opts }} {{ pi }} "bash -lc 'cd {{ pi_ws }} && source tools/ros-env.sh --overlay && timeout -s INT {{ seconds }} ros2 run pimesh_hello hello_node --ros-args -p rate_hz:=2.0 -p text:=${marker}'"
    wait || true

# P3 gate: one source tree, two distros, messages across the LAN
gate-hello-lan:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    source "{{ ws }}/tools/ros-env.sh" --overlay
    echo "== gate-hello-lan =="

    node_pat='[p]imesh_hello/lib/pimesh_hello'
    cleanup() {
        pkill -f "$node_pat" 2>/dev/null || true
        ssh {{ ssh_opts }} {{ pi }} "bash -lc 'pkill -f \"$node_pat\" || true'" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM HUP

    pi_env() { ssh {{ ssh_opts }} {{ pi }} "bash -lc 'echo \$$1'"; }

    # 1. Preflight. Both hosts on the same domain with the same middleware, or
    #    every later number in this gate is measuring the wrong graph.
    here_domain=${ROS_DOMAIN_ID:-unset}; here_rmw=${RMW_IMPLEMENTATION:-unset}
    pi_domain=$(pi_env ROS_DOMAIN_ID); pi_rmw=$(pi_env RMW_IMPLEMENTATION)
    for pair in "dev:$here_domain:42" "pi:$pi_domain:42" \
                "dev:$here_rmw:rmw_cyclonedds_cpp" "pi:$pi_rmw:rmw_cyclonedds_cpp"; do
        IFS=: read -r who got want <<<"$pair"
        [[ $got == "$want" ]] || { echo "FAIL: ${who} has ${got}, expected ${want}"; exit 1; }
    done

    # The daemon caches the discovery graph and will happily show a stale one,
    # which is how a fix that worked looks like a fix that did not.
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
    ssh {{ ssh_opts }} {{ pi }} "bash -lc 'ros2 daemon stop; ros2 daemon start'" >/dev/null 2>&1 || true

    # 2. The same source builds on both, from source, on each machine's own ROS.
    t0=$(date +%s.%N); {{ just_executable() }} build >/dev/null; t1=$(date +%s.%N)
    here_build=$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.1f", b - a }')
    t0=$(date +%s.%N); {{ just_executable() }} build-pi >/dev/null; t1=$(date +%s.%N)
    pi_build=$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.1f", b - a }')

    # 3. Different distros, or the cross-distro property was never exercised.
    here_distro=$ROS_DISTRO
    pi_distro=$(pi_env ROS_DISTRO)
    if [[ $here_distro == "$pi_distro" ]]; then
        echo "FAIL: both hosts report ${here_distro} — nothing cross-distro was tested"
        exit 1
    fi

    # 4. Twenty seconds of 2 Hz across the LAN.
    #
    # The payload carries the Pi's hostname because `ros2 topic info -v` will
    # not: it reports the publisher's *node* name, which is `hello_node` no
    # matter which machine it runs on. Provenance has to be in the message.
    pi_host=$(ssh {{ ssh_opts }} {{ pi }} hostname)
    marker="hello-from-${pi_host}"
    log=$(mktemp -d)/lan.log

    if pgrep -f "$node_pat" >/dev/null 2>&1; then
        echo "FAIL: a pimesh_hello node is already running here — the count would be a lie"
        exit 1
    fi

    timeout -s INT 40 ros2 run pimesh_hello echo_node >"$log" 2>&1 &
    sleep 3
    ssh {{ ssh_opts }} {{ pi }} "bash -lc 'cd {{ pi_ws }} && source tools/ros-env.sh --overlay && timeout -s INT 20 ros2 run pimesh_hello hello_node --ros-args -p rate_hz:=2.0 -p text:=${marker}'" >/dev/null 2>&1 &
    pi_job=$!

    sleep 10
    info=$(ros2 topic info -v /hello_node/hello 2>&1 || true)
    pub_count=$(awk '/^Publisher count:/ { print $3 }' <<<"$info")

    wait "$pi_job" 2>/dev/null || true
    sleep 2
    cleanup

    received=$(grep -c 'echo n=' "$log" || true)
    payload=$(grep -m1 -o 'payload=0x[0-9a-f]* "[^"]*"' "$log" | sed 's/.*"\(.*\)"/\1/' || true)

    if [[ ${pub_count:-0} -ne 1 ]]; then
        echo "FAIL: publisher count was ${pub_count:-none}, expected 1"; exit 1
    fi
    if [[ $payload != "$marker" ]]; then
        echo "FAIL: payload \"${payload}\" is not \"${marker}\" — that is not the Pi's talker"
        exit 1
    fi
    if [[ $received -lt 34 ]]; then
        echo "FAIL: received ${received} messages in 20 s at 2 Hz, budget is 34 of 40"
        exit 1
    fi

    echo
    echo "distros          : dev=${here_distro}  pi=${pi_distro}  (assert different)"
    echo "build times      : dev=${here_build}s  pi=${pi_build}s  (same source, both from source)"
    echo "domain / rmw     : ${here_domain} / ${here_rmw} on both hosts"
    echo "publisher count  : ${pub_count}  (assert 1)"
    echo "payload          : \"${payload}\"  (assert \"${marker}\")"
    echo "received         : ${received} of 40 in 20 s at 2 Hz  (assert >= 34)"
    echo "PASS gate-hello-lan"

# Run the composed container here. seconds = how long to run it
hello-compose seconds="30":
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    source "{{ ws }}/tools/ros-env.sh" --overlay

    cleanup() {
        pkill -f 'rclcpp_components/[c]omponent_container' 2>/dev/null || true
        sleep 0.5
        pkill -f 'ros2 launch pimesh_[a-z]*' 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM HUP

    timeout -s INT {{ seconds }} ros2 launch pimesh_hello hello.launch.py

# P4 gate: Ctrl-C and a closed window both leave nothing running, on either machine
gate-hello-clean:
    #!/usr/bin/env bash
    set -euo pipefail
    cd "{{ ws }}"
    source "{{ ws }}/tools/ros-env.sh" --overlay
    echo "== gate-hello-clean =="

    log=$(mktemp -d)/clean.log

    # Start with a clean slate, or the gate cannot tell its own leak from
    # somebody else's.
    if ! {{ just_executable() }} stragglers >/dev/null 2>&1; then
        echo "FAIL: something was already running before the gate started"
        {{ just_executable() }} stragglers || true
        exit 1
    fi

    # setsid so the session gets its own process group. That is what makes the
    # kill below realistic: a terminal sends Ctrl-C to the foreground *group*,
    # not to one pid, and a trap that only covers the pid it was installed on
    # would pass a weaker test than the one it has to survive.
    run_and_signal() {          # $1 = INT or HUP
        setsid {{ just_executable() }} hello-lan 45 >"$log" 2>&1 &
        local launcher=$! pgid up=0
        pgid=$(ps -o pgid= -p "$launcher" | tr -d ' ')

        # Both ends must actually be up, or killing them proves nothing.
        for _ in $(seq 45); do
            if pgrep -f '/lib/[p]imesh_hello/' >/dev/null 2>&1 &&
               ssh {{ ssh_opts }} {{ pi }} "bash -lc 'pgrep -f \"/lib/[p]imesh_[a-z]*/\"'" \
                   >/dev/null 2>&1; then
                up=1; break
            fi
            sleep 1
        done
        if [[ $up -ne 1 ]]; then
            echo "FAIL: the session never got both ends running, so there was nothing to kill"
            tail -20 "$log"; return 1
        fi

        kill -"$1" -"$pgid" 2>/dev/null || true
        sleep 3
        return 0
    }

    counts=""
    for sig in INT HUP; do
        run_and_signal "$sig"
        # `just stragglers` is the assertion: it prints a count per host and
        # exits non-zero if any survived.
        out=$({{ just_executable() }} stragglers 2>&1) && rc=0 || rc=$?
        echo "--- after SIG${sig} ---"
        echo "$out"
        if [[ $rc -ne 0 ]]; then
            echo "FAIL: SIG${sig} left processes behind"
            exit 1
        fi
        counts+="SIG${sig}: $(grep -o '[0-9]*$' <<<"$out" | tr '\n' '/' | sed 's:/$::')  "
    done

    echo
    echo "survivors dev/pi : ${counts}(assert 0/0 after each signal)"
    echo "PASS gate-hello-clean"
