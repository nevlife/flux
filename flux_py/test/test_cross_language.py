"""C++ and Python meeting on one live channel, in two processes.

Everything else that compares the bindings compares them within one process, or compares bytes
through a file (flux_gen's test_emit). Neither observes the step before the bytes: agreeing on
which segment to open. That step is built from the process domain, the channel key and the
fingerprint, and each binding resolves it on its own -- which is how the two ended up in
different domains, meeting nothing, reporting nothing. Nothing in
the suite would have shown it, because nothing made a C++ process and a Python process meet.

The C++ side links flux_core directly rather than going through flux_cpp. That is the pairing
that broke: flux_py against a plain flux_core user, with no ROS node on either end to agree
through.
"""

import os
import shutil
import subprocess
import sys
import time

import numpy as np
import pytest

import flux

CORE_INCLUDE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "flux_core", "include"))


def _core_lib():
    env = os.environ.get("FLUX_CORE_LIB")
    if env and os.path.isfile(env):
        return env
    ws = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    for rel in ("install/flux_core/lib/libflux_core.a", "build/flux_core/libflux_core.a"):
        cand = os.path.join(ws, rel)
        if os.path.isfile(cand):
            return cand
    return None


CORE_LIB = _core_lib()

needs_cxx = pytest.mark.skipif(
    shutil.which("g++") is None or not os.path.isdir(CORE_INCLUDE) or CORE_LIB is None,
    reason="flux-cap:cxx-toolchain")

FP = 0xC0FFEE01
KEY = f"/pytest/xlang/{os.getpid()}"

SUBSCRIBER = r"""
#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include <cstdio>
#include <cstdint>
#include <string>

// argv: <key> <fingerprint> <expected bytes>. Prints the resolved names first so a failure to
// meet is readable as a name mismatch rather than as a timeout.
int main(int argc, char ** argv)
{
  const std::string key = argv[1];
  const std::uint64_t fp = std::stoull(argv[2]);
  const std::size_t want = static_cast<std::size_t>(std::stoul(argv[3]));
  std::printf("domain %s\n", flux::process_domain().c_str());
  const std::string sp = flux::signpost_name(key, fp);
  std::printf("signpost %s\n", sp.c_str());
  std::fflush(stdout);
  auto ch = flux::Channel::open(sp, fp);
  for (int i = 0; i < 500; ++i) {
    flux::FrameView v = ch.take_blocking(20'000'000);
    if (!v) continue;
    const std::uint8_t * p = static_cast<const std::uint8_t *>(v.data());
    std::uint64_t sum = 0;
    for (std::size_t k = 0; k < v.size(); ++k) sum += p[k];
    std::printf("frame %zu %llu\n", v.size(), static_cast<unsigned long long>(sum));
    std::fflush(stdout);
    return v.size() == want ? 0 : 2;
  }
  std::printf("timeout\n");
  return 1;
}
"""

PUBLISHER = r"""
#include "flux/channel.hpp"
#include "flux/discovery.hpp"
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>
#include <unistd.h>

// argv: <key> <fingerprint> <nbytes> <fill>. Publishes until the parent has taken one, so the
// subscriber may attach after this starts.
int main(int argc, char ** argv)
{
  const std::string key = argv[1];
  const std::uint64_t fp = std::stoull(argv[2]);
  const std::size_t n = static_cast<std::size_t>(std::stoul(argv[3]));
  const std::uint8_t fill = static_cast<std::uint8_t>(std::stoul(argv[4]));
  std::printf("domain %s\n", flux::process_domain().c_str());
  const std::string sp = flux::signpost_name(key, fp);
  std::printf("signpost %s\n", sp.c_str());
  std::fflush(stdout);
  auto ch = flux::Channel::create(sp, 1 << 16, 4, fp);
  std::vector<std::uint8_t> buf(n, fill);
  std::printf("ready\n");
  std::fflush(stdout);
  for (int i = 0; i < 500; ++i) {
    ch.publish(buf.data(), buf.size());
    ::usleep(10000);
  }
  return 0;
}
"""


def _build(tmp_path, source, name):
    src, exe = tmp_path / f"{name}.cpp", tmp_path / name
    src.write_text(source)
    subprocess.run(
        ["g++", "-std=c++17", "-O1", f"-I{CORE_INCLUDE}", str(src), CORE_LIB, "-pthread",
         "-o", str(exe)],
        check=True, capture_output=True, text=True)
    return exe


def _domain_env(value):
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = value
    env.pop("FLUX_DOMAIN", None)
    return env


@needs_cxx
def test_a_cpp_process_reads_what_a_python_process_published(tmp_path):
    exe = _build(tmp_path, SUBSCRIBER, "xsub")
    key = KEY + "/py2cpp"
    payload = np.arange(256, dtype=np.uint8)

    pub = flux.Publisher(key, slot_size=1 << 16, slot_count=4, fingerprint=FP)
    p = subprocess.Popen(
        [str(exe), key, str(FP), str(payload.nbytes)], stdout=subprocess.PIPE, text=True)
    try:
        assert p.stdout.readline().startswith("domain ")
        assert p.stdout.readline().startswith("signpost ")
        deadline = time.time() + 10
        while time.time() < deadline and p.poll() is None:
            assert pub.publish(payload) == flux.Published.Ok
            time.sleep(0.01)
        out, _ = p.communicate(timeout=5)
    finally:
        if p.poll() is None:
            p.kill()
    assert p.returncode == 0, f"the C++ subscriber never met the Python publisher: {out!r}"
    assert out.split()[:3] == ["frame", str(payload.nbytes), str(int(payload.sum()))], out


@needs_cxx
def test_a_python_process_reads_what_a_cpp_process_published(tmp_path):
    exe = _build(tmp_path, PUBLISHER, "xpub")
    key = KEY + "/cpp2py"
    n, fill = 512, 9

    p = subprocess.Popen(
        [str(exe), key, str(FP), str(n), str(fill)], stdout=subprocess.PIPE, text=True)
    try:
        for _ in range(3):
            assert p.stdout.readline() != "", "the C++ publisher died before it was ready"
        sub = flux.Subscription(key, fingerprint=FP)
        view = sub.take_blocking(5_000_000_000)
        assert view is not None, "the Python subscriber never met the C++ publisher"
        got = np.asarray(view)
        assert got.nbytes == n
        assert int(got.min()) == fill and int(got.max()) == fill
    finally:
        p.kill()
        p.wait(timeout=5)


@needs_cxx
def test_the_two_meet_under_a_non_default_ros_domain_id(tmp_path):
    # The regression the audit's D4 had no test for. Both ends default their domain from
    # ROS_DOMAIN_ID; when only one did, they resolved "7" and "0", opened two segments and each
    # reported a healthy endpoint with nothing on it. Meeting is the assertion -- a wrong domain
    # on either side shows up here as a timeout, not as an error either process can raise.
    exe = _build(tmp_path, SUBSCRIBER, "xsub7")
    key = KEY + "/domain7"
    payload = np.full(64, 3, dtype=np.uint8)
    env = _domain_env("7")

    parent = subprocess.Popen(
        [sys.executable, "-c",
         "import os, time, numpy as np, flux\n"
         f"pub = flux.Publisher({key!r}, slot_size=1 << 16, slot_count=4, fingerprint={FP})\n"
         "print(pub.domain, flush=True)\n"
         "print(pub.segment_name, flush=True)\n"
         f"buf = np.full({payload.size}, {int(payload[0])}, dtype=np.uint8)\n"
         "end = time.time() + 10\n"
         "while time.time() < end:\n"
         "    pub.publish(buf)\n"
         "    time.sleep(0.01)\n"],
        stdout=subprocess.PIPE, text=True, env=env)
    child = None
    try:
        # Read the publisher's announcement before starting the subscriber: a signpost resolved
        # before the publisher writes it names a segment that is not there yet.
        py_domain = parent.stdout.readline().strip()
        py_segment = parent.stdout.readline().strip()
        child = subprocess.Popen(
            [str(exe), key, str(FP), str(payload.nbytes)],
            stdout=subprocess.PIPE, text=True, env=env)
        cxx_domain = child.stdout.readline().split(maxsplit=1)[1].strip()
        child.stdout.readline()  # signpost
        out, _ = child.communicate(timeout=15)
    finally:
        for p in (parent, child):
            if p is not None and p.poll() is None:
                p.kill()
    assert py_domain == "7", f"the Python endpoint resolved domain {py_domain!r}"
    assert cxx_domain == "7", f"the C++ endpoint resolved domain {cxx_domain!r}"
    assert "7" in py_segment, py_segment
    assert child.returncode == 0, f"the two ends did not meet in domain 7: {out!r}"
