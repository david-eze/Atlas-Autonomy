"""Unit tests for semantic command parsing and coordinate resolution."""

import unittest


class SemanticResolver:
    def __init__(self):
        self.locations = {
            'charging_station': (2.0, 1.5),
            'workstation': (6.0, 4.0),
            'storage_area': (1.0, 8.0),
            'office': (9.0, 2.0),
        }

    def parse_command(self, command: str) -> str:
        lowered = command.strip().lower()
        for verb in ('go to ', 'navigate to ', 'drive to ', 'take me to '):
            if lowered.startswith(verb):
                lowered = lowered[len(verb):]
                break
        if lowered.startswith('the '):
            lowered = lowered[4:]
        return lowered.strip()

    def resolve(self, command: str):
        target = self.parse_command(command)
        return self.locations.get(target)


class TestSemanticNav(unittest.TestCase):
    def setUp(self):
        self.resolver = SemanticResolver()

    def test_parse_command_variants(self):
        self.assertEqual(self.resolver.parse_command("Go to the workstation"), "workstation")
        self.assertEqual(self.resolver.parse_command("Navigate to charging_station"), "charging_station")
        self.assertEqual(self.resolver.parse_command("Drive to the storage_area"), "storage_area")

    def test_resolve_coordinates(self):
        coords = self.resolver.resolve("Go to the workstation")
        self.assertEqual(coords, (6.0, 4.0))

    def test_unknown_target(self):
        coords = self.resolver.resolve("Go to the moon")
        self.assertIsNone(coords)


if __name__ == '__main__':
    unittest.main()
