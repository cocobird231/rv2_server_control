"""Check source precedence and safe argument handling without invoking a build."""

import importlib.util
import pathlib
import subprocess
import unittest.mock

import pytest


@pytest.fixture
def wrapper():
    """Load the source entry point without running its command-line main."""
    path = pathlib.Path(__file__).resolve().parents[2] / "scripts/build_with_unitree.py"
    spec = importlib.util.spec_from_file_location("unitree_build", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def workspace(wrapper, tmp_path, monkeypatch):
    """Provide a discoverable owner while making all subprocesses explicit mocks."""
    sources = {wrapper.SERVER: [tmp_path / wrapper.SERVER]}
    monkeypatch.setattr(
        wrapper, "discover_packages", unittest.mock.Mock(return_value=sources)
    )
    monkeypatch.setattr(
        wrapper, "underlay_prefix", unittest.mock.Mock(return_value=None)
    )
    return tmp_path, sources


def test_workspace_source_precedes_underlay_and_bundle(wrapper, workspace):
    """Source overlays win without probing or adding another Unitree provider."""
    root, sources = workspace
    for name in wrapper.UNITREE_PACKAGES:
        sources[name] = [root / name]
    wrapper.underlay_prefix.side_effect = AssertionError("source must win")
    command, providers = wrapper.build_command([root], [])
    assert command == [
        "colcon",
        "build",
        "--base-paths",
        str(root),
        "--packages-up-to",
        wrapper.SERVER,
    ]
    assert providers == {
        name: f"workspace source {root / name}" for name in wrapper.UNITREE_PACKAGES
    }


def test_underlay_precedes_bundle(wrapper, workspace):
    """An already sourced installation avoids rebuilding either vendored package."""
    root, _ = workspace
    wrapper.underlay_prefix.return_value = root / "installed_unitree"
    command, providers = wrapper.build_command([root], [])
    for name in wrapper.UNITREE_PACKAGES:
        assert str(wrapper.VENDORED_ROOT / name) not in command
        assert providers[name] == f"sourced underlay {root / 'installed_unitree'}"
    assert wrapper.underlay_prefix.call_args_list == [
        unittest.mock.call(name) for name in wrapper.UNITREE_PACKAGES
    ]


@pytest.mark.parametrize(
    "api_provider, go_provider",
    [
        ("source", "underlay"),
        ("underlay", "source"),
        ("source", "vendor"),
        ("vendor", "source"),
        ("underlay", "vendor"),
        ("vendor", "underlay"),
    ],
)
def test_mixed_providers_are_selected_independently(
    wrapper, workspace, api_provider, go_provider
):
    """The provider of one package must never hide or override the other's source."""
    root, sources = workspace
    choices = dict(zip(wrapper.UNITREE_PACKAGES, (api_provider, go_provider)))
    for name, choice in choices.items():
        if choice == "source":
            sources[name] = [root / name]
    wrapper.underlay_prefix.side_effect = lambda name: (
        root / "installed" / name if choices[name] == "underlay" else None
    )
    command, providers = wrapper.build_command([root], [])
    roots = command[
        command.index("--base-paths") + 1 : command.index("--packages-up-to")
    ]
    assert roots == [
        str(root),
        *(
            str(wrapper.VENDORED_ROOT / name)
            for name, choice in choices.items()
            if choice == "vendor"
        ),
    ]
    for name, choice in choices.items():
        if choice == "source":
            assert providers[name] == f"workspace source {root / name}"
        elif choice == "underlay":
            assert providers[name] == f"sourced underlay {root / 'installed' / name}"
        else:
            assert providers[name] == f"bundled source {wrapper.VENDORED_ROOT / name}"
    assert wrapper.underlay_prefix.call_args_list == [
        unittest.mock.call(name)
        for name, choice in choices.items()
        if choice != "source"
    ]


def test_fallback_preserves_paths_and_forwarded_argument_tokens(wrapper, workspace):
    """Preserve shell tokens; add API/Go roots without scanning the vendor parent."""
    root, _ = workspace
    spaced = root / "extra source"
    spaced.mkdir()
    forwarded = ["--symlink-install", "--cmake-args", "-DLABEL=a b;$(false)"]
    command, providers = wrapper.build_command([root, spaced, root], forwarded)
    assert command == [
        "colcon",
        "build",
        "--base-paths",
        str(root),
        str(spaced),
        *(str(wrapper.VENDORED_ROOT / name) for name in wrapper.UNITREE_PACKAGES),
        "--packages-up-to",
        wrapper.SERVER,
        *forwarded,
    ]
    assert all("bundled source" in provider for provider in providers.values())
    assert str(wrapper.VENDORED_ROOT) not in command
    assert str(wrapper.VENDORED_ROOT / "unitree_hg") not in command


@pytest.mark.parametrize("name", ["unitree_api", "unitree_go", "rv2_server_control"])
def test_duplicate_source_packages_fail(wrapper, workspace, name):
    """Ambiguous copies must not silently choose one package."""
    root, sources = workspace
    sources[name] = [root / "copy_a", root / "copy_b"]
    with pytest.raises(ValueError, match=f"Duplicate {name}"):
        wrapper.build_command([root], [])
    wrapper.underlay_prefix.assert_not_called()


@pytest.mark.parametrize("missing", ["unitree_api", "unitree_go"])
def test_missing_bundle_fails_without_starting_build(
    wrapper, workspace, monkeypatch, missing
):
    """Missing one required bundle cannot silently produce a partial build plan."""
    root, sources = workspace
    for name in wrapper.UNITREE_PACKAGES:
        if name != missing:
            sources[name] = [root / name]
    monkeypatch.setattr(wrapper, "VENDORED_ROOT", root / "missing vendor")
    build = unittest.mock.Mock()
    monkeypatch.setattr(wrapper.subprocess, "call", build)
    assert wrapper.main(["--source-root", str(root)]) == 2
    build.assert_not_called()


def test_underlay_index_failure_is_not_treated_as_missing_package(
    wrapper, tmp_path, monkeypatch
):
    """Only PackageNotFoundError permits fallback; malformed index errors propagate."""
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path))
    lookup = unittest.mock.Mock(side_effect=ValueError("broken package index"))
    monkeypatch.setattr(wrapper.ament_packages, "get_package_prefix", lookup)
    with pytest.raises(ValueError, match="broken package index"):
        wrapper.underlay_prefix("unitree_go")
    lookup.assert_called_once_with("unitree_go")


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
    build = unittest.mock.Mock()
    monkeypatch.setattr(wrapper.subprocess, "call", build)
    assert wrapper.main(["--source-root", str(root)]) == 19
    build.assert_not_called()
    wrapper.underlay_prefix.assert_not_called()


def test_dry_run_and_build_exit_status(wrapper, workspace, monkeypatch, capsys):
    """Dry-run prints the decision; a real invocation retains colcon's status."""
    root, _ = workspace
    build = unittest.mock.Mock(return_value=7)
    monkeypatch.setattr(wrapper.subprocess, "call", build)
    args = ["--source-root", str(root)]
    assert wrapper.main([*args, "--dry-run", "--", "--symlink-install"]) == 0
    build.assert_not_called()
    output = capsys.readouterr().out
    for name in wrapper.UNITREE_PACKAGES:
        assert f"{name}: bundled source" in output
    assert wrapper.main([*args, "--", "--symlink-install"]) == 7
    assert build.call_args.args[0][-1] == "--symlink-install"
