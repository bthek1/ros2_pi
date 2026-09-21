"""The C++ and the JavaScript agree only because somebody typed it twice.

`dashboard_node.cpp` writes a JSON object; `web/app.js` reads fields off it by
name. `web_server.hpp` numbers five channels; `app.js` numbers them again in a
literal. Nothing in the build relates either pair, nothing fails when they
disagree, and both failures are *quiet in the same way*: the page renders, the
socket stays up, the pipeline is untouched, and one panel is blank or one number
is a permanent em dash.

CLAUDE.md names this shape for `volume_key` — two nodes that have to hold the
same value, a mismatch that is not a partial failure but two separate worlds —
and says to write a test for every pair of keys that has to agree. These pairs
cross a language boundary, so the test reads both files as text. That is not
elegant and it is the only thing that can check it at all: the reader is
hand-written JavaScript that runs in a browser and cannot be linked against.

Source-reading, hermetic, no node, no socket, no browser — so it runs on both
machines like every other suite here.
"""

import re
from pathlib import Path

import pytest

PACKAGE = Path(__file__).resolve().parent.parent
SRC = PACKAGE.parent

APP_JS = (PACKAGE / "web" / "app.js").read_text()
WEB_SERVER_HPP = (PACKAGE / "include" / "pimesh_dashboard" / "web_server.hpp").read_text()
DASHBOARD_NODE_CPP = (PACKAGE / "src" / "dashboard_node.cpp").read_text()
DASHBOARD_PROBE_CPP = (PACKAGE / "src" / "dashboard_probe.cpp").read_text()


# ---------------------------------------------------------------------------
# The channel byte
# ---------------------------------------------------------------------------

def cpp_channels():
    """`Channel` as web_server.hpp declares it: name -> number."""
    body = re.search(
        r"enum class Channel\s*:\s*std::uint8_t\s*\{(.*?)\}", WEB_SERVER_HPP, re.S)
    assert body, "the Channel enum has moved or changed shape"
    return {
        name.lower(): int(value)
        for name, value in re.findall(r"(\w+)\s*=\s*(\d+)\s*,", body.group(1))
    }


def js_channels():
    """`CH` as app.js declares it: name -> number."""
    body = re.search(r"const CH = \{(.*?)\};", APP_JS, re.S)
    assert body, "the CH table has moved or changed shape"
    return {
        name.lower(): int(value)
        for name, value in re.findall(r"(\w+)\s*:\s*(\d+)", body.group(1))
    }


def test_both_ends_number_the_channels_the_same():
    """A swapped pair puts JPEG bytes through JSON.parse and blanks two panels.

    Not a crash: `onmessage` throws inside a handler nobody catches, the socket
    survives, and the stage table and the depth strip both stop updating while
    the pipeline runs perfectly. The gate would still pass — it reads the
    channels it asked for, by number, from the same wrong table.
    """
    assert cpp_channels() == js_channels()


def test_no_channel_is_zero():
    """Zero is the value a byte has when nobody set it.

    `broadcast` prepends `static_cast<uint8_t>(channel)`, so a channel numbered
    0 and a zero byte from a framing bug are the same message. Starting at 1
    means the page's `default: break` catches the second case.
    """
    assert 0 not in cpp_channels().values()


def test_every_channel_the_server_can_send_is_handled_by_the_page():
    """An unhandled channel is bandwidth spent on a panel that does not exist."""
    handled = {name.lower() for name in re.findall(r"case CH\.(\w+):", APP_JS)}
    assert handled == set(cpp_channels())


# ---------------------------------------------------------------------------
# The JSON field names
# ---------------------------------------------------------------------------

def emitted_fields(function_name):
    """The keys one of dashboard_node.cpp's *_json() builders writes.

    They are written as string literals into an ostringstream — `"\\"rate_hz\\":"`
    — which is exactly why this is worth a test: a typo there is a field the page
    reads as `undefined`, and `undefined` renders as an em dash that looks like a
    stage which has not reported yet.
    """
    start = DASHBOARD_NODE_CPP.index(f"std::string DashboardNode::{function_name}()")
    end = DASHBOARD_NODE_CPP.index("\n}\n", start)
    return set(re.findall(r'\\"(\w+)\\":', DASHBOARD_NODE_CPP[start:end]))


def test_the_two_builders_emit_the_fields_this_test_knows_about():
    """A guard on the extraction itself, before anything is concluded from it.

    A regex that silently matched nothing would make every assertion below pass
    over an empty set — the `cost_mean=0.00` failure mode, where an unmeasured
    value and a good one have the same spelling.
    """
    assert len(emitted_fields("stats_json")) >= 10
    assert len(emitted_fields("pose_json")) >= 5
    assert "rate_hz" in emitted_fields("stats_json")
    assert "trail" in emitted_fields("pose_json")


def js_reads(variable):
    """Field names app.js takes off a parsed object held in `variable`."""
    return set(re.findall(rf"\b{variable}\.(\w+)\b", APP_JS))


def test_every_stats_field_the_page_reads_is_one_the_node_sends():
    """The failure direction. A field that is not sent reads as `undefined`.

    `fmt(undefined)` is an em dash and `esc(undefined)` is an empty cell, both
    of which are what the page shows for a stage that has legitimately not
    reported yet — so a renamed field looks exactly like a stage being quiet.
    """
    emitted = emitted_fields("stats_json")
    # `stats` is the whole object; `s` and `by.<stage>` are one row of it.
    read = js_reads("stats") | js_reads("s") | {
        field for _, field in re.findall(r"\bby\.(\w+)\.(\w+)\b", APP_JS)}
    assert read <= emitted, f"app.js reads fields stats_json does not send: {read - emitted}"


def test_every_pose_field_the_page_reads_is_one_the_node_sends():
    emitted = emitted_fields("pose_json")
    read = js_reads("pose")
    assert read <= emitted, f"app.js reads fields pose_json does not send: {read - emitted}"


def test_every_field_the_gates_instrument_scrapes_is_one_the_node_sends():
    """`dashboard_probe` decides gates/dashboard.sh, so it is part of this contract.

    It scrapes the JSON with string search rather than parsing it, which means a
    renamed field gives it *zero* rather than an error — and a gate that reports
    `ws_dropped=0` because it could not find the field asserts the pacing rule is
    working while measuring nothing. This is the gate-instrument rule that
    `test_orb_reference` exists for, one channel over.
    """
    scraped = set(re.findall(r'\\\\"(\w+)\\\\"', DASHBOARD_PROBE_CPP))
    scraped |= set(re.findall(r'"(\w+)\\":', DASHBOARD_PROBE_CPP))
    emitted = emitted_fields("stats_json") | emitted_fields("pose_json")
    assert scraped, "the probe scrapes nothing — this test is checking an empty set"
    assert scraped <= emitted, f"the probe scrapes fields nobody sends: {scraped - emitted}"


# Fields the node sends that no reader reads. Not a failure — `frames_in` and
# `frames_out` are the PipelineStats contract and belong on the wire whether or
# not this page has a column for them yet — but pinned, so that adding a field
# and forgetting to wire it up has to be acknowledged here rather than costing
# 10 Hz of bandwidth unnoticed.
UNREAD_ON_PURPOSE = {
    "latency_p95_ms",      # published by every stage; the table shows the mean
    "frames_in",           # the PipelineStats denominator
    "frames_out",
    "mesh_versions",       # a counter for a human reading the socket
    "frame",               # pose frame_id; the 3D view has only one frame
    "position",            # drawn as the last element of `trail`
    "orientation",         # no camera-attached geometry on the page yet
}


def test_the_fields_nobody_reads_are_the_ones_we_meant():
    emitted = emitted_fields("stats_json") | emitted_fields("pose_json")
    read = (js_reads("stats") | js_reads("s") | js_reads("pose") |
            {f for _, f in re.findall(r"\bby\.(\w+)\.(\w+)\b", APP_JS)} |
            set(re.findall(r'\\\\"(\w+)\\\\"', DASHBOARD_PROBE_CPP)) |
            set(re.findall(r'"(\w+)\\":', DASHBOARD_PROBE_CPP)))
    # `stages` is the array the rows come out of, read by iteration rather than
    # by name off a row.
    read.add("stages")
    assert (emitted - read) == UNREAD_ON_PURPOSE


# ---------------------------------------------------------------------------
# The stage names the page keys on
# ---------------------------------------------------------------------------

def published_stage_names():
    """Every string any node in this workspace puts in PipelineStats.stage."""
    names = set()
    for source in SRC.rglob("*.cpp"):
        names |= set(re.findall(r'stage\s*=\s*"([a-z_]+)"', source.read_text()))
    return names


def test_the_stages_the_page_names_are_stages_something_publishes():
    """The headline figures are keyed on two stage *strings*, typed twice.

    `by.depth` and `by.capture` fill the two numbers at the top of the page. A
    node renaming its stage — or a typo in either file — leaves those reading
    `—` for the life of the session, which is indistinguishable from the stage
    being down. It is the `volume_key` mismatch: not a partial failure, two
    separate worlds.
    """
    keyed = set(re.findall(r"\bby\.(\w+)\b", APP_JS))
    published = published_stage_names()
    assert keyed, "the page keys on no stage — this test is checking nothing"
    assert keyed <= published, f"the page keys on stages nobody publishes: {keyed - published}"


def test_capture_is_the_one_stage_the_dev_box_cannot_measure():
    """P10's whole point, and the reason the page has a row for it.

    `capture` comes from the Pi and carries the frames the kernel dropped before
    dequeue. If that stage name stopped being published the page would lose the
    only figure in the pipeline no dev-box node can see, and would show an em
    dash rather than an error.
    """
    assert "capture" in published_stage_names()
    camera = (SRC / "pimesh_camera" / "src" / "camera_node.cpp").read_text()
    assert 'stage = "capture"' in camera


# ---------------------------------------------------------------------------
# The binary mesh layout
# ---------------------------------------------------------------------------

def test_the_page_decodes_the_mesh_at_the_offsets_the_packer_writes():
    """`mesh_payload.hpp` says 4 / 12 / 3; app.js says 4 / 12 / 3, in literals.

    test_mesh_payload pins the packer against those numbers. This pins the
    *other* end, which no C++ test can reach. A stride read one byte out still
    yields floats, so the page draws believable geometry in the wrong places —
    a mesh that looks like a poor reconstruction, which is what this project has
    spent two milestones expecting to see.
    """
    header = re.search(r"getUint32\((\d+), true\)", APP_JS)
    assert header, "the page no longer reads a little-endian count at a fixed offset"
    assert int(header.group(1)) == 0

    mesh_payload = (PACKAGE / "include" / "pimesh_dashboard" / "mesh_payload.hpp").read_text()
    constants = dict(re.findall(
        r"constexpr std::size_t kMesh(\w+)\s*=\s*([^;]+);", mesh_payload))
    assert constants["HeaderBytes"].strip() == "4"
    assert constants["ColourStride"].strip() == "3"
    assert "3 * sizeof(float)" in constants["PositionStride"]

    # The page slices positions from [4, 4 + count*12) and colours from the end
    # of that to + count*3. Both offsets appear as arithmetic on `count`.
    assert re.search(r"byteOffset \+ 4, .*byteOffset \+ 4 \+ count \* 12", APP_JS, re.S)
    assert re.search(r"count \* 12, .*count \* 12 \+ count \* 3", APP_JS, re.S)


def test_the_page_copies_the_float_array_rather_than_viewing_it():
    """A `Float32Array` view needs 4-byte alignment and this one cannot have it.

    The positions start at offset 4 of a payload whose own offset in the frame
    is 1 — the channel byte — so the float block is at an odd address. A
    `subarray` there throws a RangeError in `onmessage`, killing the 3D view and
    nothing else; `slice` copies and is correct. It is one word apart and the
    smaller word is the one that looks more efficient.
    """
    mesh = APP_JS[APP_JS.index("function onMesh"):]
    mesh = mesh[:mesh.index("\n}\n")]
    # Comments only, stripped — app.js explains this choice in a comment that
    # names the word being searched for, and a test that reads prose as code is
    # a test that passes when somebody deletes the comment.
    code = "\n".join(re.sub(r"//.*", "", line) for line in mesh.splitlines())
    assert "new Float32Array(bytes.buffer.slice(" in code
    assert "subarray" not in code, "a Float32Array subarray at an odd offset throws"


@pytest.mark.parametrize("triangles", [1, 2, 40000, 120000])
def test_both_ends_compute_the_same_payload_length(triangles):
    """The arithmetic itself, done independently from each file's own constants."""
    vertices = triangles * 3
    packed = 4 + vertices * (3 * 4 + 3)
    # What app.js will read: header, then count*12, then count*3.
    decoded = 4 + vertices * 12 + vertices * 3
    assert packed == decoded
