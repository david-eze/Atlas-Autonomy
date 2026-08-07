"""Merge an environment-specific Nav2 profile over the office base.

The warehouse and challenging YAML files only override the parameters
that differ from the office baseline, keeping configuration DRY.
Nav2 itself only accepts a single params file, so this utility merges
the dictionaries recursively at launch time and writes the result to a
temporary file.

Usage:
    python3 profile_merger.py nav2_office.yaml nav2_warehouse.yaml /tmp/nav2_merged.yaml
"""

from __future__ import annotations

import copy
import sys
from typing import Any, Dict

import yaml


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Recursively merge ``override`` into ``base``.

    Dictionaries are merged key-by-key; all other values (including
    lists, which Nav2 treats as atomic parameter values) are overwritten.
    """
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def merge_profiles(base_path: str, override_path: str, output_path: str) -> None:
    with open(base_path, 'r', encoding='utf-8') as f:
        base = yaml.safe_load(f)
    with open(override_path, 'r', encoding='utf-8') as f:
        override = yaml.safe_load(f)

    merged = deep_merge(base, override)
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(merged, f, sort_keys=False)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print('Usage: profile_merger.py <base> <override> <output>', file=sys.stderr)
        sys.exit(1)
    merge_profiles(sys.argv[1], sys.argv[2], sys.argv[3])
