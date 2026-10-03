"""Offline synchronization regressions. No SSH or robot install is touched."""

import importlib.machinery
import importlib.util
import io
import os
from pathlib import Path
import shutil
import socket
import time

import pytest


@pytest.fixture
def sync(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[1] / "sync-remote"
    loader = importlib.machinery.SourceFileLoader("sync_remote_test_module", str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    monkeypatch.setattr(module, "SOCKET_PATH", str(tmp_path / "sync.sock"))
    return module


@pytest.mark.parametrize("message", [
    b"Nothing to do: replicas have not changed since last sync.\n",
    b"Synchronization complete at 05:00:00  (600 items transferred, 0 skipped, 0 failed)\n",
])
def test_completion_survives_every_possible_read_boundary(sync, message):
    for boundary in range(len(message) + 1):
        progress = sync.SyncProgress()
        progress.feed(b"Scanning...\r\n" + message[:boundary])
        progress.feed(message[boundary:])
        assert progress.ready


def test_new_work_errors_and_partial_lines_invalidate_old_completion(sync):
    for following in (
        b"Scanning changes\n", b"Error: Sys_blocked_io\n", b"Error: partial",
        b"Synchronization complete at now (1 items transferred, 1 skipped, 0 failed)\n",
        b"Synchronization complete at now (1 items transferred, 0 skipped, 1 failed)\n",
    ):
        progress = sync.SyncProgress()
        progress.feed(b"Nothing to do: replicas agree\n" + following)
        assert not progress.ready


def test_spool_accepts_nonblocking_stdout_burst_without_a_reader(sync, tmp_path, monkeypatch):
    executable = tmp_path / "unison"
    executable.write_text(
        "#!/usr/bin/python3\nimport os,stat\n"
        "assert stat.S_ISREG(os.fstat(1).st_mode)\n"
        "os.set_blocking(1, False)\n"
        "os.write(1, b'x' * 1048576)\n"
        "os.write(2, b'\\nNothing to do: replicas agree\\n')\n"
    )
    executable.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    process, reader = sync.create_process()
    try:
        assert process.wait(timeout=5) == 0  # Deliberately do not drain while it writes.
        assert reader.read() == b"x" * 1048576 + b"\nNothing to do: replicas agree\n"
    finally:
        sync.stop_process(process)
        reader.close()


@pytest.mark.parametrize("code,log,expected,reply", [
    (2, b"Nothing to do: replicas agree\n", 2, b""),
    (3, b"Error: Sys_blocked_io\n", 3, b""),
    (0, b"Scanning...\n", 1, b""),
    (0, b"Nothing to do: replicas agree\n", 0, b"f"),
])
def test_child_exit_and_completion_control_wait_sync_reply(
    sync, monkeypatch, code, log, expected, reply
):
    class ExitedProcess:
        def poll(self):
            return code

    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(2)

    def create():
        client.connect(sync.SOCKET_PATH)
        return ExitedProcess(), io.BytesIO(log)

    monkeypatch.setattr(sync, "create_process", create)
    monkeypatch.setattr(sync, "write_output", lambda _: None)
    try:
        assert sync.main() == expected
        assert client.recv(1) == reply
        assert not Path(sync.SOCKET_PATH).exists()
    finally:
        client.close()


def test_forwarding_retries_nonblocking_and_short_writes(sync, monkeypatch):
    calls = 0
    written = bytearray()

    def write(fd, data):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise BlockingIOError()
        written.extend(data[:3])
        return min(3, len(data))

    monkeypatch.setattr(sync.os, "write", write)
    monkeypatch.setattr(sync.select, "select", lambda *_: ([], [1], []))
    sync.write_output(b"complete log output")
    assert written == b"complete log output"


@pytest.mark.skipif(shutil.which("unison") is None, reason="real Unison unavailable")
def test_real_unison_syncs_many_changes_and_next_watch_cycle(sync, tmp_path, monkeypatch):
    source, destination, state = (tmp_path / name for name in ("source", "destination", "state"))
    for directory in (source, destination, state):
        directory.mkdir()
    for index in range(600):
        name = f"sample_{index:04d}_wheel_leg_identification_sample__struct.hpp"
        (source / name).write_text("new contents\n")
        (source / name).chmod(0o644)
        (destination / name).write_text("old contents\n")
        (destination / name).chmod(0o600)
    monkeypatch.setattr(sync, "SRC_DIR", str(source))
    monkeypatch.setattr(sync, "DST_DIR", str(destination))
    monkeypatch.setenv("UNISON", str(state))
    process, reader = sync.create_process()
    progress = sync.SyncProgress()

    def await_sync():
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            data = reader.read()
            if data:
                assert b"Sys_blocked_io" not in data
                progress.feed(data)
            assert process.poll() is None
            if progress.ready:
                return
            time.sleep(.05)
        pytest.fail("Unison did not finish the local-only test sync")

    try:
        time.sleep(.4)  # Slow terminal: let the producer run without a reader.
        await_sync()
        for original in source.iterdir():
            copy = destination / original.name
            assert copy.read_bytes() == original.read_bytes()
            assert copy.stat().st_mode & 0o777 == original.stat().st_mode & 0o777
        changed = next(source.iterdir())
        changed.write_text("second watched update\n")
        progress.ready = False
        await_sync()
        assert (destination / changed.name).read_bytes() == changed.read_bytes()
    finally:
        sync.stop_process(process)
        reader.close()
