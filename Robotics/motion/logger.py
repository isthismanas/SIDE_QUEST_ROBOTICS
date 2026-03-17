"""Minimal structured console logger for robotics modules."""

from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from threading import Lock
from typing import Any

from robot_config import LOG_LEVEL, LOG_MODULES, RUN_MODE


_LEVEL_ORDER = {
	"DEBUG": 10,
	"INFO": 20,
	"WARN": 30,
	"ERROR": 40,
	"QUIET": 50,
}

_LEGACY_PREFIXES = (
	"[CONTROL]",
	"[ADMIN]",
	"[CAM]",
	"[DOBOT]",
	"[STACK]",
	"[DRIFT]",
)

_JSON_LOG_LOCK = Lock()
_PERCEPTION_LOG_DIR = os.path.normpath(
	os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "perception", "logs")
)


def _normalize_level(level: str) -> str:
	return (level or "").upper().strip()


def _normalize_module(module: str) -> str:
	return (module or "").upper().strip()


def _module_level(module: str) -> str:
	module_key = _normalize_module(module)
	if module_key in LOG_MODULES:
		return _normalize_level(LOG_MODULES[module_key])
	return _normalize_level(LOG_LEVEL)


def is_enabled(module: str, level: str) -> bool:
	module_tag = _normalize_module(module)
	message_level = _LEVEL_ORDER.get(_normalize_level(level))
	if message_level is None:
		return False

	if RUN_MODE == "COMP":
		return message_level >= _LEVEL_ORDER["INFO"]

	threshold = _LEVEL_ORDER.get(_module_level(module_tag), _LEVEL_ORDER["INFO"])
	return message_level >= threshold and threshold < _LEVEL_ORDER["QUIET"]


def _strip_leading_module_prefix(module_tag: str, msg: str) -> str:
	text = msg
	prefix = f"[{module_tag}]"
	if text.startswith(prefix):
		return text[len(prefix):].lstrip()

	for legacy_prefix in _LEGACY_PREFIXES:
		if text.startswith(legacy_prefix):
			return text[len(legacy_prefix):].lstrip()

	return text


def _timestamp_utc() -> str:
	return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def write_jsonl_event(stream_name: str, event: dict[str, Any]) -> str:
	os.makedirs(_PERCEPTION_LOG_DIR, exist_ok=True)
	file_path = os.path.join(_PERCEPTION_LOG_DIR, f"{stream_name}.jsonl")
	record = {"ts_utc": _timestamp_utc(), **event}
	line = json.dumps(record, separators=(",", ":"), sort_keys=True)
	with _JSON_LOG_LOCK:
		with open(file_path, "a", encoding="utf-8") as handle:
			handle.write(line)
			handle.write("\n")
	return file_path


def log(module: str, level: str, msg: str) -> None:
	if is_enabled(module, level):
		module_tag = _normalize_module(module) or "UNKNOWN"
		clean_msg = _strip_leading_module_prefix(module_tag, msg)

		if RUN_MODE == "COMP":
			message_level = _LEVEL_ORDER.get(_normalize_level(level))
			if message_level is None:
				return
			if message_level == _LEVEL_ORDER["DEBUG"]:
				return
			if message_level < _LEVEL_ORDER["WARN"]:
				if (
					"READY:" not in clean_msg
					and "successfully placed" not in clean_msg
					and "Ready to START" not in clean_msg
				):
					return

		print(f"[{module_tag}] {clean_msg}")


def debug(module: str, msg: str) -> None:
	log(module, "DEBUG", msg)


def info(module: str, msg: str) -> None:
	log(module, "INFO", msg)


def warn(module: str, msg: str) -> None:
	log(module, "WARN", msg)


def error(module: str, msg: str) -> None:
	log(module, "ERROR", msg)
