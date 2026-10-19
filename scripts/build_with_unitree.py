#!/usr/bin/env python3
"""Build this server, selecting each required Unitree package independently."""

import argparse
import os
import pathlib
import shlex
import subprocess
import sys

from ament_index_python import packages as ament_packages

SERVER = "rv2_server_control"
UNITREE_PACKAGES = ("unitree_api", "unitree_go")
VENDORED_ROOT = pathlib.Path(__file__).resolve().parents[1] / "thirdparty/unitree"


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
        packages.setdefault(name, []).append(pathlib.Path(path).resolve())
    return packages


def underlay_prefix(package):
    """Find this package in the environment the caller has sourced."""
    if not os.environ.get("AMENT_PREFIX_PATH"):
        return None
    try:
        return pathlib.Path(ament_packages.get_package_prefix(package))
    except ament_packages.PackageNotFoundError:
        return None


def validate_build_arguments(arguments):
    """Keep discovery and dependency selection under this wrapper's control."""
    reserved = {
        "--base-paths",
        "--paths",
        "--metas",
        "--ignore-user-meta",
        "--mixin",
        "--mixin-files",
        "--packages-",
    }
    for argument in arguments:
        option = argument.split("=", 1)[0]
        # colcon's argparse also accepts unambiguous long-option abbreviations.
        if option.startswith("--") and (
            option.startswith("--packages-")
            or any(reserved_option.startswith(option) for reserved_option in reserved)
        ):
            raise ValueError(
                f"{option} changes package discovery or selection; use --source-root "
                "for roots. This wrapper always builds --packages-up-to "
                f"{SERVER}; use colcon directly for other selections."
            )


def build_command(source_roots, arguments):
    """Choose each package's provider and construct a dependency-aware command."""
    validate_build_arguments(arguments)
    roots = list(dict.fromkeys(pathlib.Path(root).resolve() for root in source_roots))
    for root in roots:
        if not root.is_dir():
            raise ValueError(f"Source root does not exist: {root}")
    packages = discover_packages(roots)
    for name in (SERVER, *UNITREE_PACKAGES):
        paths = packages.get(name, [])
        if len(paths) > 1:
            raise ValueError(
                f"Duplicate {name} source packages: {', '.join(map(str, paths))}"
            )
    if SERVER not in packages:
        raise ValueError(
            f"{SERVER} was not discovered; include its source via --source-root"
        )
    providers = {}
    for name in UNITREE_PACKAGES:
        if name in packages:
            providers[name] = f"workspace source {packages[name][0]}"
        elif (prefix := underlay_prefix(name)) is not None:
            providers[name] = f"sourced underlay {prefix}"
        else:
            vendor = VENDORED_ROOT / name
            if not (vendor / "package.xml").is_file():
                raise ValueError(f"Bundled {name} is missing: {vendor}")
            roots.append(vendor)
            providers[name] = f"bundled source {vendor}"
    command = [
        "colcon",
        "build",
        "--base-paths",
        *map(str, roots),
        "--packages-up-to",
        SERVER,
        *arguments,
    ]
    return command, providers


def main(argv=None):
    """Run from a workspace root, forwarding options after the -- delimiter."""
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Example: %(prog)s -- --symlink-install --cmake-args -DBUILD_TESTING=OFF",
    )
    parser.add_argument(
        "--source-root",
        action="append",
        metavar="PATH",
        help="Recursive source root, repeatable (default: ./src); quote paths with spaces",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="Discover and print; do not build"
    )
    parser.add_argument(
        "build_arguments", nargs=argparse.REMAINDER, metavar="-- BUILD_OPTIONS"
    )
    options = parser.parse_args(argv)
    arguments = options.build_arguments
    if arguments:
        if arguments[0] != "--":
            parser.error("Pass colcon build options after --")
        arguments = arguments[1:]
    try:
        command, providers = build_command(options.source_root or ["src"], arguments)
        for package, provider in providers.items():
            print(f"{package}: {provider}", flush=True)
        print(shlex.join(command), flush=True)
        if options.dry_run:
            return 0
        return subprocess.call(command)
    except subprocess.CalledProcessError as error:
        print(
            f"colcon discovery failed (exit {error.returncode}); build not started",
            file=sys.stderr,
        )
        return error.returncode
    except (OSError, ValueError) as error:
        print(f"{parser.prog}: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
