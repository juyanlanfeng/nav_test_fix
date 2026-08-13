#!/usr/bin/env python3
"""Small helpers for preserving reproducible conversion provenance.

Generated fields always win.  Human or runtime validation fields are carried
forward only while the generated artifact hash is unchanged, so rerunning a
deterministic build does not erase evidence and changing geometry cannot make
old evidence look current.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


JsonObject = dict[str, Any]


def sha256_file(path: Path) -> str:
    """Return the SHA-256 of a potentially large source file."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json_object(path: Path) -> JsonObject:
    """Load one JSON object, returning an empty object when it does not exist."""
    if not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object in {path}")
    return value


def write_json_object(path: Path, value: JsonObject) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def merge_conversion_root(previous: JsonObject, generated: JsonObject) -> JsonObject:
    """Keep downstream provenance only for the exact same source STEP bytes.

    A missing hash is deliberately treated as unknown rather than compatible.
    This prevents a new or replaced STEP from inheriting old PLY/PCD validation
    evidence merely because it was converted into the same output directory.
    """
    previous_hash = previous.get("source_step_sha256")
    generated_hash = generated.get("source_step_sha256")
    if previous_hash and previous_hash == generated_hash:
        return {**previous, **generated}
    return generated


def merge_hashed_section(
    metadata: JsonObject,
    section_name: str,
    generated: JsonObject,
    hash_key: str,
) -> None:
    """Merge unknown evidence fields only for byte-identical artifacts."""
    previous = metadata.get(section_name)
    if (
        isinstance(previous, dict)
        and previous.get(hash_key)
        and previous.get(hash_key) == generated.get(hash_key)
    ):
        metadata[section_name] = {**previous, **generated}
    else:
        metadata[section_name] = generated


def merge_hashed_object(
    previous: JsonObject,
    generated: JsonObject,
    hash_keys: tuple[str, ...],
) -> JsonObject:
    """Preserve report evidence only when old and new artifact bytes match.

    ``hash_keys`` may contain aliases used by different report schema versions,
    for example the historical ``sha256`` and current ``output_sha256`` names.
    Generated fields always replace their previous counterparts.
    """

    def artifact_hash(value: JsonObject) -> Any:
        return next((value.get(key) for key in hash_keys if value.get(key)), None)

    previous_hash = artifact_hash(previous)
    generated_hash = artifact_hash(generated)
    if previous_hash and previous_hash == generated_hash:
        return {**previous, **generated}
    return generated


def relative_or_absolute(path: Path, base: Path) -> str:
    """Use portable paths inside a conversion tree and absolute paths outside."""
    try:
        return str(path.resolve().relative_to(base.resolve()))
    except ValueError:
        return str(path.resolve())
