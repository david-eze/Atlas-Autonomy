"""Semantic location lookup: friendly names -> map coordinates.

Maps object categories (from YOLO detections) and predefined place
names to coordinates. These coordinates only ever get consumed by
the mission layer to build a Nav2 goal, never by control.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class SemanticLocation:
    """A named navigable location with a 2D map pose."""

    name: str
    x: float
    y: float
    category: str = ''
    confidence: float = 1.0


class SemanticMap:
    def __init__(self, initial_locations: Optional[Dict[str, SemanticLocation]] = None) -> None:
        self._locations = dict(initial_locations or {})

    def register(self, location: SemanticLocation) -> None:
        self._locations[location.name.lower()] = location

    def lookup(self, name: str) -> Optional[SemanticLocation]:
        """Resolve a friendly name to a location, e.g. 'the charging station' -> 'charging_station'."""
        key = name.strip().lower()
        for prefix in ('the ', 'a ', 'an '):
            if key.startswith(prefix):
                key = key[len(prefix):]
                break
        key = key.replace(' ', '_')
        if key.endswith('s') and key[:-1] in self._locations:
            key = key[:-1]
        return self._locations.get(key)

    def update_from_detection(
        self,
        category: str,
        x: float,
        y: float,
        confidence: float,
    ) -> Optional[SemanticLocation]:
        """Upsert a location for a detected object category.

        Landmark categories (chair, pallet, table, etc.) get registered
        so 'go to the chair' resolves later.
        """
        category_key = category.strip().lower().replace(' ', '_')
        if category_key in ('person', 'dog', 'cat', 'bicycle', 'motorbike'):
            return None
        loc = SemanticLocation(
            name=category_key,
            x=x,
            y=y,
            category=category_key,
            confidence=confidence,
        )
        self._locations[category_key] = loc
        return loc

    def all(self) -> Dict[str, SemanticLocation]:
        return dict(self._locations)
