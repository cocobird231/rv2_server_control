#!/usr/bin/env python3
"""Build this server, preferring workspace or sourced Unitree over bundled API."""

import argparse
import os
import shlex
import subprocess
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix, PackageNotFoundError

SERVER = "rv2_server_control"
API = "unitree_api"
VENDORED_API = Path(__file__).resolve().parents[1] / "thirdparty/unitree/unitree_api"


def discover_packages(roots):
    """Use colcon's actual discovery and retain every path for duplicate checks."""
    result = subprocess.run(
        ["colcon", "list", "--base-paths", *map(str, roots)],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    )
    packages = {}
    for line in result.stdout.splitlines():
        name, path, _ = line.split("\t", 2)
        packages.setdefault(name, []).append(Path(path).resolve())
    return packages


def underlay_prefix():
    """Find the first installed API in the environment the caller has sourced."""
    if not os.environ.get("AMENT_PREFIX_PATH"):
        return None
    try:
        return Path(get_package_prefix(API))
    except PackageNotFoundError:
        return None


def validate_build_arguments(arguments):
    """Keep discovery and dependency selection under this wrapper's control."""
    reserved = {"--base-paths", "--paths", "--metas", "--ignore-user-meta", "--mixin"}
    for argument in arguments:
        option = argument.split("=", 1)[0]
        if option in reserved or option.startswith("--packages-"):
            raise ValueError(
                f"{option} changes package discovery or selection; use --source-root "
                "for roots. This wrapper always builds --packages-up-to "
                f"{SERVER}; use colcon directly for other selections."
            )


def build_command(source_roots, arguments):
    """Choose one API provider and construct a dependency-aware colcon command."""
    validate_build_arguments(arguments)
    roots = list(dict.fromkeys(Path(root).resolve() for root in source_roots))
    for root in roots:
        if not root.is_dir():
            raise ValueError(f"Source root does not exist: {root}")
    packages = discover_packages(roots)
    for name in (SERVER, API):
        paths = packages.get(name, [])
        if len(paths) > 1:
            raise ValueError(f"Duplicate {name} source packages: {', '.join(map(str, paths))}")
    if SERVER not in packages:
        raise ValueError(f"{SERVER} was not discovered; include its source via --source-root")
    if API in packages:
        provider = f"workspace source {packages[API][0]}"
    elif (prefix := underlay_prefix()) is not None:
        provider = f"sourced underlay {prefix}"
    else:
        if not (VENDORED_API / "package.xml").is_file():
            raise ValueError(f"Bundled {API} is missing: {VENDORED_API}")
        roots.append(VENDORED_API)
        provider = f"bundled source {VENDORED_API}"
    command = [
        "colcon", "build", "--base-paths", *map(str, roots),
        "--packages-up-to", SERVER, *arguments,
    ]
    return command, provider


def main(argv=None):
    """Run from a workspace root, forwarding options after the -- delimiter."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Example: %(prog)s -- --symlink-install --cmake-args -DBUILD_TESTING=OFF",
    )
    parser.add_argument(
        "--source-root", action="append", metavar="PATH",
        help="Recursive source root, repeatable (default: ./src); quote paths with spaces",
    )
    parser.add_argument("--dry-run", action="store_true", help="Discover and print; do not build")
    parser.add_argument("build_arguments", nargs=argparse.REMAINDER, metavar="-- BUILD_OPTIONS")
    options = parser.parse_args(argv)
    arguments = options.build_arguments
    if arguments:
        if arguments[0] != "--":
            parser.error("Pass colcon build options after --")
        arguments = arguments[1:]
    try:
        command, provider = build_command(options.source_root or ["src"], arguments)
        print(f"{API}: {provider}", flush=True)
        print(shlex.join(command), flush=True)
        if options.dry_run:
            return 0
        return subprocess.call(command)
    except subprocess.CalledProcessError as error:
        print(f"colcon discovery failed (exit {error.returncode}); build not started", file=sys.stderr)
        return error.returncode
    except (OSError, ValueError) as error:
        print(f"{parser.prog}: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
