# Testing

*Current as of 2026-09-04: 10 gtest cases in `pimesh_camera` and 13 pytest
cases for the gate tools, plus three gates. All passing on both machines.*

## Two layers, and they answer different questions

| | **Tests** | **Gates** |
| --- | --- | --- |
| Ask | is this logic right? | does the real system do what we claim? |
| Need hardware | **never** | yes — camera, Wi-Fi, two machines |
| Run with | `just test`, `just test-pi` | `just gate-build`, `just gate-capture`, `just gate-provision` |
| Take | under a second | 1–4 minutes |
| Live in | `src/*/test/`, `tools/test_*.py` | the justfile, one per plan phase |

Both are required and neither substitutes for the other. The unit tests would
happily pass on a machine with no camera attached; the gates are the only thing
that can tell you the camera is delivering frames at the rate the hardware
allows. Equally, a gate that fails tells you *something* is wrong across two
machines and a radio link — the tests are what make that bisectable.

**Every phase of [the bootstrap plan](../plans/in-progress/bootstrap-plan.md)
ends in a gate.** Tests are added alongside whatever logic the phase introduces
that can be tested without hardware.

## What is tested today

### `pimesh_camera` — 10 cases, `src/pimesh_camera/test/test_v4l2_capture.cpp`

Run on **both** machines, so nothing in them may require a device — the dev box
has no camera.

- **The timestamp conversion (5 cases).** This is the most consequential logic
  in the package and the reason the node exists at all, so it was pulled out of
  `wait_frame` into a free function, `to_system_clock_ns`, purely so it could be
  tested. The cases assert the offset arithmetic, that a frame's *age* is
  preserved, that the answer does not depend on when the clock pair was
  sampled — and one case **reproduces the usb_cam 0.8.1 epoch bug** in
  arithmetic and shows it landing ~0.72 s late where ours is exact.
- **Timestamp provenance (2 cases).** The `V4L2_BUF_FLAG_TIMESTAMP_*` values
  are restated as literals, so a change in `<videodev2.h>` becomes a test
  failure rather than silently wrong provenance.
- **Failure paths (3 cases).** A missing device, a regular file, and a
  character device that is not V4L2 (`/dev/null`) must each throw with the path
  and the errno in the message. These matter most and would otherwise only be
  exercised by accident.

### The gate tools — 13 cases, `tools/test_check_capture.py`

`check_capture.py` decides whether P1 passes, so it gets tested like anything
else that can say "everything is fine". The cases drive its CLI and check both
the exit code and the output: a healthy run passes, dropping a third of the
frames fails, a 3% sampling difference does **not** fail (or the gate cries wolf
every run), a collapsed delivered rate fails, and — the one that matters —
**feeding it usb_cam's measured 0.223 s / 0.362 s offsets makes it fail**.

There is also a case asserting that a *missing* hardware measurement fails
rather than quietly passing on the strength of the checks that could still run.

## Rules

- **A test suite that has never failed is not evidence.** Break the thing it
  covers, watch the suite go red, put it back. Done for the timestamp
  conversion on 2026-09-04: adding 1 ms to `to_system_clock_ns` turns 4 of the
  10 cases red, and removing it turns them green again.
- **If logic is hard to test, that is a fact about the code.** The stamp
  conversion was three lines inside a 90-line method that needs a camera; it is
  now a free function with the reasoning in a comment above it. The refactor
  was worth more than the tests.
- **Tests run on both distros.** `just test-pi` builds and runs the same cases
  under Jazzy on aarch64 with g++ 13.3.0. A test that has only ever run on
  Lyrical says nothing about the machine that actually runs the camera.
- **No test may need hardware.** The moment one does, it is a gate.
- **Name the case after the claim**, not the function:
  `DoesNotReproduceTheUsbCamEpochBug`, not `TestConvert2`. A failing test name
  should tell you what broke without opening the file.

## What is deliberately not run

**The `ament_lint_auto` linters** (copyright, cpplint, uncrustify). The
generated `package.xml` files declared them and nothing used them, so the
declarations were removed on 2026-09-04 rather than left as decoration — a
`test_depend` should name something that runs.

They are worth turning on, and the reason they are not on yet is honest rather
than principled: `ament_copyright` wants a header on every file and a `LICENSE`
in every package, and `uncrustify` would reformat code that is currently
readable. That is a change to make deliberately, in its own commit, not as a
side effect of adding the first real tests. If it happens, it belongs in the
future file with a trigger.

## Adding a test

C++ goes beside the code it covers:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_<thing> test/test_<thing>.cpp)
  target_link_libraries(test_<thing> <the library under test>)
endif()
```

Python for the repo's own tools goes in `tools/test_*.py` and is picked up by
`just test`'s pytest pass — `tools/` is not a ROS package, so colcon cannot see
it, which is why the recipe runs both.

## In the editor

The pytest suite appears in VS Code's Testing sidebar once `just venv` has been
run — the workspace points the Python extension at `.venv`, a
`--system-site-packages` pointer at `/usr/bin/python3`, because naming the
interpreter in settings alone was silently overridden and discovery failed with
`No module named pytest`
([setup.md](setup.md#working-in-vs-code)). The C++ cases run from the
`test` task or the debugger — see
[setup.md](setup.md#working-in-vs-code).

## Running

```bash
just venv        # once: the interpreter the editor and pytest both use
just test        # colcon (gtest) + pytest, dev box, ~1 s
just test-pi     # the same gtest cases on the Pi, under Jazzy
just gate-build      # P0: builds on both distros, interfaces identical
just gate-capture    # P1: the camera, against real hardware
just gate-provision  # P9: the playbook is idempotent and the Pi matches
```

`just test` exits non-zero if either suite fails, and reports both rather than
stopping at the first.
