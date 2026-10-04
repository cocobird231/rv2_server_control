"""Check source precedence and safe argument handling without invoking a build."""

import subprocess
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from unittest.mock import Mock

import pytest


@pytest.fixture
def wrapper():
    """Load the source entry point without running its command-line main."""
    path = Path(__file__).resolve().parents[2] / "scripts/build_with_unitree.py"
    spec = spec_from_file_location("unitree_build", path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def workspace(wrapper, tmp_path, monkeypatch):
    """Provide a discoverable owner while making all subprocesses explicit mocks."""
    sources = {wrapper.SERVER: [tmp_path / wrapper.SERVER]}
    monkeypatch.setattr(wrapper, "discover_packages", Mock(return_value=sources))
    monkeypatch.setattr(wrapper, "underlay_prefix", Mock(return_value=None))
    return tmp_path, sources


def test_workspace_source_precedes_underlay_and_bundle(wrapper, workspace):
    """A source overlay wins without probing or adding another API provider."""
    root, sources = workspace
    sources[wrapper.API] = [root / "external_api"]
    wrapper.underlay_prefix.side_effect = AssertionError("source must win")
    command, provider = wrapper.build_command([root], [])
    assert command == [
        "colcon",
        "build",
        "--base-paths",
        str(root),
        "--packages-up-to",
        wrapper.SERVER,
    ]
    assert "workspace source" in provider


def test_underlay_precedes_bundle(wrapper, workspace):
    """An already sourced installation avoids rebuilding the vendored API."""
    root, _ = workspace
    wrapper.underlay_prefix.return_value = root / "installed_api"
    command, provider = wrapper.build_command([root], [])
    assert str(wrapper.VENDORED_API) not in command
    assert provider == f"sourced underlay {root / 'installed_api'}"


def test_fallback_preserves_paths_and_forwarded_argument_tokens(wrapper, workspace):
    """Spaces and shell metacharacters remain data, with only API added as a root."""
    root, _ = workspace
    spaced = root / "extra source"
    spaced.mkdir()
    forwarded = ["--symlink-install", "--cmake-args", "-DLABEL=a b;$(false)"]
    command, provider = wrapper.build_command([root, spaced, root], forwarded)
    assert command == [
        "colcon",
        "build",
        "--base-paths",
        str(root),
        str(spaced),
        str(wrapper.VENDORED_API),
        "--packages-up-to",
        wrapper.SERVER,
        *forwarded,
    ]
    assert "bundled source" in provider


@pytest.mark.parametrize("name", ["unitree_api", "rv2_server_control"])
def test_duplicate_source_packages_fail(wrapper, workspace, name):
    """Ambiguous copies must not silently choose one package."""
    root, sources = workspace
    sources[name] = [root / "copy_a", root / "copy_b"]
    with pytest.raises(ValueError, match=f"Duplicate {name}"):
        wrapper.build_command([root], [])


@pytest.mark.parametrize(
    "option",
    [
        "--base-paths=other",
        "--paths",
        "--metas",
        "--mixin",
        "--packages-select",
        "--packages-skip=unitree_api",
        "--packages-up-to",
        "--base",
        "--path=other",
        "--meta",
        "--pack",
        "--mixin-files",
    ],
)
def test_discovery_and_selection_overrides_rejected(wrapper, workspace, option):
    """Forwarded arguments cannot hide or skip the required fallback package."""
    root, _ = workspace
    with pytest.raises(ValueError, match="changes package discovery or selection"):
        wrapper.build_command([root], [option])
    wrapper.discover_packages.assert_not_called()


def test_missing_owner_or_source_root_fail(wrapper, workspace):
    """Wrong working directories fail before an unrelated workspace gets built."""
    root, sources = workspace
    with pytest.raises(ValueError, match="Source root does not exist"):
        wrapper.build_command([root / "missing"], [])
    sources.clear()
    with pytest.raises(ValueError, match="was not discovered"):
        wrapper.build_command([root], [])


def test_discovery_failure_is_not_treated_as_missing_api(
    wrapper, workspace, monkeypatch
):
    """Propagate colcon failure and never start a build or probe fallback."""
    root, _ = workspace
    wrapper.discover_packages.side_effect = subprocess.CalledProcessError(
        19, ["colcon", "list"]
    )
    build = Mock()
    monkeypatch.setattr(wrapper.subprocess, "call", build)
    assert wrapper.main(["--source-root", str(root)]) == 19
    build.assert_not_called()
    wrapper.underlay_prefix.assert_not_called()


def test_dry_run_and_build_exit_status(wrapper, workspace, monkeypatch, capsys):
    """Dry-run prints the decision; a real invocation retains colcon's status."""
    root, _ = workspace
    build = Mock(return_value=7)
    monkeypatch.setattr(wrapper.subprocess, "call", build)
    args = ["--source-root", str(root)]
    assert wrapper.main([*args, "--dry-run", "--", "--symlink-install"]) == 0
    build.assert_not_called()
    assert "bundled source" in capsys.readouterr().out
    assert wrapper.main([*args, "--", "--symlink-install"]) == 7
    assert build.call_args.args[0][-1] == "--symlink-install"
