# Future — deferred out of the hello-world plan

Companion to the hello-world plan, [issue #2](https://github.com/bthek1/ros2_pi/issues/2).

Everything here is **not executable yet**, which is why it is not a phase. Each
entry names the **trigger** that would make it executable. When a trigger fires,
the entry is **deleted from this file** and appended to that issue's body as the
next unused phase number, with a test — see [../README.md](../README.md).

"Later" is not a trigger. If an entry's trigger is not something that can be
observed happening, it is not written down properly yet.

---

## Select the component container executable per distro

**What.** `launch/hello.launch.py` hard-codes `executable='component_container_mt'`.
Replace it with a per-distro choice — `component_container --executor-type
multi-threaded` where that exists, `component_container_mt` where it does not —
so one launch file keeps working on both machines.

**Why not now.** Measured 2026-09-08: Lyrical prints `This executable is
deprecated and will be removed in M-turtle. Use 'component_container
--executor-type multi-threaded' instead`, but Jazzy ships no `--executor-type`
flag. `component_container_mt` exists on **both** distros today, so it is the
only spelling that is currently correct on both, and any per-distro branch would
be dead code on one side with nothing to test it against.

**Trigger.** Either machine reaching a distro where `component_container_mt` is
absent — M-turtle on the dev box, or the Pi moving off Jazzy. At that point the
launch file breaks on one machine and the branch has two real cases to cover.

---

## A standalone-versus-composed equivalence gate

**What.** Assert that `ros2 run pimesh_hello hello_node` and the same component
loaded into a container produce identical topic names, parameter sets and
message content — so "a node that only works standalone is a bug" becomes a
check rather than a convention.

**Why not now.** With one trivially-parameterised component pair the check is
close to tautological: `gate-hello-talk` runs the standalone form and
`gate-hello-ipc` runs the composed form, and both already assert the rate, the
payload and the parameter. A dedicated equivalence gate would restate them.

**Trigger.** The first component whose behaviour depends on how it is hosted —
concretely, the first node that takes a `NodeOptions` argument it did not
declare itself, or the first one whose standalone main grows past init /
construct / spin / shutdown. `pimesh_camera` is the likely first.
