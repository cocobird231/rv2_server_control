"""Check fallback planning against real colcon discovery, without building nodes."""

import importlib.util
import pathlib
import shutil
import subprocess

import pytest

OWNER = pathlib.Path(__file__).resolve().parents[2]


@pytest.fixture
def wrapper():
    """Load the same command users invoke from their workspace root."""
    spec = importlib.util.spec_from_file_location(
        "unitree_build", OWNER / "scripts/build_with_unitree.py"
    )
    module = importlib.util.module_from_spec(spec)
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


def add_underlay_marker(prefix, name):
    """Create the real ament index entry needed to discover an installed package."""
    marker = prefix / "share/ament_index/resource_index/packages" / name
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.touch()


def test_nested_api_needs_explicit_fallback_root(wrapper, workspace):
    """Discovery stops at the owner; explicit API/Go paths bypass the parent marker."""
    original = wrapper.discover_packages([workspace])
    assert set(original) == {"rv2_server_control"}
    command, providers = wrapper.build_command([workspace], [])
    assert all("bundled source" in provider for provider in providers.values())
    assert command_packages(command) == {
        "rv2_server_control",
        "unitree_api",
        "unitree_go",
    }
    assert (OWNER / "thirdparty/unitree/AMENT_IGNORE").is_file()
    for name in wrapper.UNITREE_PACKAGES:
        assert str(wrapper.VENDORED_ROOT / name) in command
        assert not (wrapper.VENDORED_ROOT / name / "AMENT_IGNORE").exists()
    assert str(wrapper.VENDORED_ROOT) not in command
    assert str(wrapper.VENDORED_ROOT / "unitree_hg") not in command


@pytest.mark.parametrize("name", ["unitree_api", "unitree_go"])
def test_workspace_api_wins_over_sourced_underlay(
    wrapper, workspace, tmp_path, monkeypatch, name
):
    """One package's workspace source wins while the other uses its underlay."""
    external = workspace / name
    shutil.copytree(wrapper.VENDORED_ROOT / name, external)
    prefix = tmp_path / "underlay"
    for package in wrapper.UNITREE_PACKAGES:
        add_underlay_marker(prefix, package)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    command, providers = wrapper.build_command([workspace], [])
    assert providers[name] == f"workspace source {external}"
    for package in wrapper.UNITREE_PACKAGES:
        assert str(wrapper.VENDORED_ROOT / package) not in command
        if package != name:
            assert providers[package] == f"sourced underlay {prefix}"
    assert command_packages(command) == {"rv2_server_control", name}


def test_sourced_underlay_suppresses_fallback(
    wrapper, workspace, tmp_path, monkeypatch
):
    """Real ament-index markers select both installations without adding sources."""
    prefix = tmp_path / "underlay"
    for name in wrapper.UNITREE_PACKAGES:
        add_underlay_marker(prefix, name)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    command, providers = wrapper.build_command([workspace], [])
    for name in wrapper.UNITREE_PACKAGES:
        assert providers[name] == f"sourced underlay {prefix}"
        assert str(wrapper.VENDORED_ROOT / name) not in command
    assert command_packages(command) == {"rv2_server_control"}


@pytest.mark.parametrize("name", ["unitree_api", "unitree_go"])
def test_workspace_source_and_other_bundle_are_both_discovered(
    wrapper, workspace, name
):
    """A source override for one package leaves fallback enabled for the other."""
    external = workspace / name
    shutil.copytree(wrapper.VENDORED_ROOT / name, external)
    command, providers = wrapper.build_command([workspace], [])
    assert providers[name] == f"workspace source {external}"
    assert str(wrapper.VENDORED_ROOT / name) not in command
    for package in wrapper.UNITREE_PACKAGES:
        if package != name:
            vendor = wrapper.VENDORED_ROOT / package
            assert providers[package] == f"bundled source {vendor}"
            assert str(vendor) in command
    assert command_packages(command) == {
        "rv2_server_control",
        "unitree_api",
        "unitree_go",
    }


@pytest.mark.parametrize("name", ["unitree_api", "unitree_go"])
def test_partial_underlay_keeps_other_package_fallback(
    wrapper, workspace, tmp_path, monkeypatch, name
):
    """An API-only or Go-only underlay must not suppress the missing dependency."""
    prefix = tmp_path / "partial underlay"
    add_underlay_marker(prefix, name)
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(prefix))
    command, providers = wrapper.build_command([workspace], [])
    assert providers[name] == f"sourced underlay {prefix}"
    assert str(wrapper.VENDORED_ROOT / name) not in command
    fallback = next(package for package in wrapper.UNITREE_PACKAGES if package != name)
    vendor = wrapper.VENDORED_ROOT / fallback
    assert providers[fallback] == f"bundled source {vendor}"
    assert str(vendor) in command
    assert command_packages(command) == {"rv2_server_control", fallback}


@pytest.mark.parametrize("name", ["unitree_api", "unitree_go"])
def test_duplicate_workspace_apis_are_rejected(wrapper, workspace, name):
    """Real discovery must not turn duplicate Unitree checkouts into a fallback."""
    for directory in ("first_copy", "second_copy"):
        shutil.copytree(wrapper.VENDORED_ROOT / name, workspace / directory)
    with pytest.raises(ValueError, match=f"Duplicate {name}"):
        wrapper.build_command([workspace], [])
