"""Check fallback planning against real colcon discovery, without building nodes."""

import shutil
import subprocess
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest

OWNER = Path(__file__).resolve().parents[2]


@pytest.fixture
def wrapper():
    """Load the same command users invoke from their workspace root."""
    spec = spec_from_file_location(
        "unitree_build", OWNER / "scripts/build_with_unitree.py"
    )
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    """Isolate sources and underlays from the framework's own built dependencies."""
    root = tmp_path / "source tree"
    root.mkdir()
    (root / "rv2_server_control").symlink_to(OWNER, target_is_directory=True)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("AMENT_PREFIX_PATH", "")
    monkeypatch.setenv("COLCON_HOME", str(tmp_path / "colcon_home"))
    defaults = tmp_path / "defaults.yaml"
    defaults.write_text("{}\n")
    monkeypatch.setenv("COLCON_DEFAULTS_FILE", str(defaults))
    return root


def command_packages(command):
    """Ask colcon to list exactly the roots and selection planned for the build."""
    output = subprocess.check_output(
        ["colcon", "list", *command[2:], "--names-only"], text=True, timeout=30
    )
    return set(output.splitlines())


def test_nested_api_needs_explicit_fallback_root(wrapper, workspace):
    """Discovery stops at the owner, but an explicit API path is discoverable."""
    original = wrapper.discover_packages([workspace])
    assert set(original) == {"rv2_server_control"}
    command, provider = wrapper.build_command([workspace], [])
    assert "bundled source" in provider
    assert command_packages(command) == {"rv2_server_control", "unitree_api"}
    assert (OWNER / "thirdparty/unitree/AMENT_IGNORE").is_file()
    assert not (wrapper.VENDORED_API / "AMENT_IGNORE").exists()


def test_workspace_api_wins_over_sourced_underlay(
    wrapper, workspace, tmp_path, monkeypatch
):
    """An external source copy is the sole API selected, even with an underlay."""
    external = workspace / "unitree_api"
    shutil.copytree(wrapper.VENDORED_API, external)
    prefix = tmp_path / "underlay"
    marker = prefix / "share/ament_index/resource_index/packages/unitree_api"
    marker.parent.mkdir(parents=True)
    marker.touch()
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    command, provider = wrapper.build_command([workspace], [])
    assert provider == f"workspace source {external}"
    assert str(wrapper.VENDORED_API) not in command
    assert command_packages(command) == {"rv2_server_control", "unitree_api"}


def test_sourced_underlay_suppresses_fallback(
    wrapper, workspace, tmp_path, monkeypatch
):
    """A real ament-index marker selects the sourced API without adding sources."""
    prefix = tmp_path / "underlay"
    marker = prefix / "share/ament_index/resource_index/packages/unitree_api"
    marker.parent.mkdir(parents=True)
    marker.touch()
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    command, provider = wrapper.build_command([workspace], [])
    assert provider == f"sourced underlay {prefix}"
    assert str(wrapper.VENDORED_API) not in command
    assert command_packages(command) == {"rv2_server_control"}


def test_duplicate_workspace_apis_are_rejected(wrapper, workspace):
    """Real discovery must not turn duplicate API checkouts into a fallback."""
    for name in ("first_api", "second_api"):
        shutil.copytree(wrapper.VENDORED_API, workspace / name)
    with pytest.raises(ValueError, match="Duplicate unitree_api"):
        wrapper.build_command([workspace], [])
