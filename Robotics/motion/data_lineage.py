from __future__ import annotations

import os
import re

import robot_config as cfg


_SAFE_TAG_PATTERN = re.compile(r"[^A-Za-z0-9._-]+")


def current_data_lineage_tag() -> str:
    raw = os.environ.get("SIDE_QUEST_DATA_TAG", getattr(cfg, "DATA_LINEAGE_TAG", ""))
    cleaned = _SAFE_TAG_PATTERN.sub("_", str(raw or "").strip())
    return cleaned.strip("._-")


def tagged_stem(stem: str) -> str:
    base = str(stem).strip()
    tag = current_data_lineage_tag()
    if not tag:
        return base
    suffix = f"_{tag}"
    if base.endswith(suffix):
        return base
    return f"{base}{suffix}"


def tagged_path(path: str) -> str:
    directory = os.path.dirname(path)
    stem, ext = os.path.splitext(os.path.basename(path))
    return os.path.join(directory, f"{tagged_stem(stem)}{ext}")


def tagged_log_stream_path(log_dir: str, stream_name: str) -> str:
    return os.path.join(log_dir, f"{tagged_stem(stream_name)}.jsonl")
