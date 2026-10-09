from __future__ import annotations

import ast
import hashlib
from pathlib import Path

import pytest

from server.policy_server.artifact_identity import (
    files_sha256,
    resolve_managed_tree,
    sha256_file,
    sha256_text_file,
    tree_sha256,
)


def test_sha256_file_matches_standard_digest(tmp_path):
    path = tmp_path / "artifact.bin"
    path.write_bytes(b"piper-policy-artifact")
    assert sha256_file(path) == hashlib.sha256(b"piper-policy-artifact").hexdigest()


def test_tree_digest_is_order_independent_and_content_sensitive(tmp_path):
    root = tmp_path / "checkpoint"
    (root / "params").mkdir(parents=True)
    (root / "train_state").mkdir()
    (root / "params" / "manifest").write_bytes(b"params")
    (root / "train_state" / "manifest").write_bytes(b"state")

    first = tree_sha256(root)
    second = tree_sha256(root)
    assert first == second
    assert len(first) == 64

    (root / "train_state" / "manifest").write_bytes(b"state changed")
    assert tree_sha256(root) != first


def test_tree_digest_rejects_empty_or_missing_artifacts(tmp_path):
    empty = tmp_path / "empty"
    empty.mkdir()
    with pytest.raises(ValueError, match="empty"):
        tree_sha256(empty)
    with pytest.raises(FileNotFoundError):
        tree_sha256(tmp_path / "missing")


def test_file_digest_rejects_directory(tmp_path):
    with pytest.raises(ValueError, match="regular file"):
        sha256_file(tmp_path)


def test_text_hash_and_source_bundle_ignore_platform_line_endings(tmp_path):
    lf = tmp_path / "lf.py"
    crlf = tmp_path / "crlf.py"
    lf.write_bytes(b"first\nsecond\n")
    crlf.write_bytes(b"first\r\nsecond\r\n")
    assert sha256_text_file(lf) == sha256_text_file(crlf)
    assert files_sha256(tmp_path, ("lf.py",)) != files_sha256(tmp_path, ("crlf.py",))

    root_a = tmp_path / "a"
    root_b = tmp_path / "b"
    root_a.mkdir()
    root_b.mkdir()
    (root_a / "source.py").write_bytes(b"first\nsecond\n")
    (root_b / "source.py").write_bytes(b"first\r\nsecond\r\n")
    assert files_sha256(root_a, ("source.py",)) == files_sha256(root_b, ("source.py",))


def test_managed_dataset_symlink_must_resolve_inside_root(tmp_path):
    root = tmp_path / "incoming"
    root.mkdir()
    actual = root / "episode-data"
    actual.mkdir()
    assert resolve_managed_tree(root, "episode-data") == actual.resolve()
    with pytest.raises(ValueError, match="safe relative"):
        resolve_managed_tree(root, "../outside")

    alias = root / "local"
    alias.mkdir()
    link = alias / "episode-data"
    try:
        link.symlink_to(actual, target_is_directory=True)
    except OSError:
        pytest.skip("symlink creation is unavailable in this Windows environment")
    assert resolve_managed_tree(root, "local/episode-data") == actual.resolve()

    outside = tmp_path / "outside"
    outside.mkdir()
    escaping = alias / "escape"
    try:
        escaping.symlink_to(outside, target_is_directory=True)
    except OSError:
        pytest.skip("symlink creation is unavailable in this Windows environment")
    with pytest.raises(ValueError, match="escapes"):
        resolve_managed_tree(root, "local/escape")
