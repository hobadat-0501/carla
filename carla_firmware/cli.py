"""Single command line entry point for the CARLA firmware."""
from __future__ import annotations

import argparse
import sys

from . import analysis, profiles


def main(argv=None):
    parser = argparse.ArgumentParser(description="CARLA 0.9.15 simulation firmware")
    sub = parser.add_subparsers(dest="command", required=True)
    for name, description in (("basic", "Basic parameter collection"),
                              ("maneuver", "Repeatable maneuver test"),
                              ("realtime", "Realtime simulation or read-only attach"),
                              ("scenario", "Run a named dynamics test on the common visible test map"),
                              ("custom", "Run a user-defined sequence of speed and steering commands")):
        sub.add_parser(name, add_help=False, help=description)
    sub.add_parser("fit", add_help=False, help="Fit model parameters")
    sub.add_parser("compare", add_help=False, help="Compare model replay with CARLA runs")
    args, rest = parser.parse_known_args(argv)
    if args.command in ("basic", "maneuver", "realtime", "scenario", "custom"):
        try:
            return profiles.run(args.command, rest)
        except KeyboardInterrupt:
            print("Stopped by user; written samples remain in the output folder.")
            return 130
        except Exception as exc:
            print(f"ERROR [{type(exc).__name__}]: {exc}")
            return 1
    sys.argv = [sys.argv[0], args.command, *rest]
    return analysis.main()


if __name__ == "__main__":
    raise SystemExit(main())
