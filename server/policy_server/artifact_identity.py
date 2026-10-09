"""Content-based identities for prediction-only policy artifacts."""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Iterable


def sha256_file(path: Path) -> str:
    """Hash one regular file without loading it fully into memory."""
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"artifact must be a regular file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text_file(path: Path) -> str:
    """Hash UTF-8 text after normalizing platform-specific line endings."""
    path = Path(path)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"artifact must be a regular file: {path}")
    content = path.read_bytes().decode("utf-8")
    canonical = content.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def tree_sha256(path: Path) -> str:
    """Hash a directory by sorted relative file names and each file digest.

    Symlinks and empty trees are rejected so the resulting identity cannot
    silently escape the selected checkpoint directory or describe no files.
    """
    path = Path(path)
    if path.is_symlink():
        raise ValueError(f"artifact tree root cannot be a symlink: {path}")
    root = path.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f"artifact tree must be a directory: {root}")
    entries: list[tuple[str, Path]] = []
    for item in root.rglob("*"):
        if item.is_symlink():
            raise ValueError(f"artifact tree contains a symlink: {item}")
        if item.is_file():
            entries.append((item.relative_to(root).as_posix(), item))
    if not entries:
        raise ValueError(f"artifact tree is empty: {root}")

    digest = hashlib.sha256()
    for relative, item in sorted(entries):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256_file(item).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def resolve_managed_tree(root: Path, relative_path: str) -> Path:
    """Resolve a dataset path, allowing only symlinks that stay inside its root."""
    base = Path(root).resolve(strict=True)
    if not base.is_dir():
        raise ValueError(f"managed artifact root must be a directory: {base}")
    relative = Path(relative_path)
    if (relative.is_absolute() or not relative.parts
            or ".." in relative.parts):
        raise ValueError("managed artifact path must be a safe relative path")
    candidate = base / relative
    resolved = candidate.resolve(strict=True)
    if not resolved.is_dir():
        raise ValueError(f"managed artifact must resolve to a directory: {resolved}")
    try:
        resolved.relative_to(base)
    except ValueError as exc:
        raise ValueError("managed artifact symlink escapes its configured root") from exc
    return resolved


def files_sha256(root: Path, relative_paths: Iterable[str]) -> str:
    """Hash a source bundle, normalizing line endings for cross-OS stability."""
    root = Path(root)
    if root.is_symlink():
        raise ValueError(f"source root cannot be a symlink: {root}")
    root = root.resolve(strict=True)
    if not root.is_dir():
        raise ValueError(f"source root must be a directory: {root}")
    entries: list[tuple[str, Path]] = []
    seen: set[str] = set()
    for name in relative_paths:
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts or not relative.parts:
            raise ValueError(f"source path must stay below root: {name}")
        normalized = relative.as_posix()
        if normalized in seen:
            raise ValueError(f"duplicate source path: {normalized}")
        seen.add(normalized)
        path = root / relative
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"source must be a regular file: {path}")
        entries.append((normalized, path))
    if not entries:
        raise ValueError("source file bundle is empty")

    digest = hashlib.sha256()
    for relative, path in sorted(entries):
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(sha256_text_file(path).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()
