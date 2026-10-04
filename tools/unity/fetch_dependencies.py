"""Fetch the RoboRacer scene's referenced assets into a sparse AutoDRIVE clone."""

import argparse
import json
from pathlib import Path
import re
import subprocess


GUID = re.compile(rb"guid:\s*([0-9a-f]{32})")
TEXT_ASSETS = {".unity", ".prefab", ".asset", ".mat", ".controller", ".overridecontroller",
               ".anim", ".shadergraph", ".shadersubgraph", ".vfx", ".rendertexture",
               ".physicmaterial", ".lighting", ".meta"}
SCENE = "Assets/Scenes/RoboRacer - Sim Racing.unity"


def git(project, *args, input_text=None):
    """Run Git inside the given Unity checkout and return its text output.

    Pass optional stdin directly and raise on command failure so an incomplete fetch cannot
    be reported as successful.
    """
    return subprocess.run(["git", "-c", "core.quotepath=false", *args], cwd=project,
                          input=input_text, encoding="utf-8", check=True,
                          stdout=subprocess.PIPE).stdout


def main():
    """Fetch the racing scene's referenced assets into an existing sparse Unity checkout.

    Build a GUID-to-file index from metadata, follow scene/material/settings references, and
    expand the sparse checkout until dependencies stop growing. Restore only missing tracked
    metadata for this lookup, remove newly restored orphan metadata afterward, and save a
    dependency report.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    args = parser.parse_args()
    project = args.project.resolve()
    if not (project / ".git").is_dir():
        parser.error("Expected an existing AutoDRIVE source clone")
    tracked = set(git(project, "ls-tree", "-r", "--name-only", "HEAD").splitlines())
    if SCENE not in tracked:
        parser.error("The checkout must use the AutoDRIVE-Simulator branch")
    initial = ["/ProjectSettings/", "/Packages/", "/Assets/Scripts/",
               "/Assets/Plugins/SocketIO/", "/Assets/Plugins/RGLUnity Plugin/",
               "/Assets/Plugins/SimpleFileBrowser/",
               "/" + SCENE, "*.meta"]
    # Additive sparse patterns preserve any assets already fetched by the user.
    git(project, "sparse-checkout", "add", "--stdin", input_text="\n".join(initial) + "\n")
    # Unity removes orphan metadata during sparse imports. Restore only missing
    # tracked metadata to rebuild the GUID index, preserving all existing edits.
    restored_meta = sorted(p for p in tracked if p.endswith(".meta")
                           and not (project / p).exists())
    if restored_meta:
        git(project, "restore", "--source=HEAD", "--worktree", "--pathspec-from-file=-",
            "--pathspec-file-nul", input_text="\0".join(restored_meta) + "\0")
    by_guid = {}
    for name in sorted(p for p in tracked if p.endswith(".meta")):
        filename = project / name
        if not filename.exists():
            raise RuntimeError(f"Missing source metadata: {name}")
        match = re.search(rb"(?m)^guid:\s*([0-9a-f]{32})", filename.read_bytes())
        if match:
            by_guid[match[1].decode()] = name[:-5]
    pending = {SCENE}
    pending.update(p for p in tracked if p.startswith("ProjectSettings/"))
    pending.update(p for p in tracked if p.startswith(("Assets/Scripts/",
                   "Assets/Plugins/SocketIO/", "Assets/Plugins/RGLUnity Plugin/",
                   "Assets/Plugins/SimpleFileBrowser/")))
    visited, unresolved = set(), set()
    for iteration in range(30):
        pending -= visited
        if not pending:
            break
        missing = {p for p in pending if p in tracked and not (project / p).exists()}
        if missing:
            print(f"Dependency pass {iteration + 1}: fetching {len(missing)} assets", flush=True)
            git(project, "sparse-checkout", "add", "--stdin",
                input_text="\n".join("/" + p for p in sorted(missing)) + "\n")
        references = set()
        for name in sorted(pending):
            visited.add(name)
            for filename in (project / name, project / (name + ".meta")):
                if (not filename.is_file() or filename.suffix.lower() not in TEXT_ASSETS
                        or filename.stat().st_size > 32 * 1024 * 1024):
                    continue
                contents = filename.read_bytes()
                if filename.name == "EditorBuildSettings.asset":
                    # Include XR/input settings without fetching every other scene.
                    contents = contents.split(b"m_configObjects:", 1)[-1]
                for match in GUID.finditer(contents):
                    guid = match[1].decode()
                    target = by_guid.get(guid)
                    if target in tracked:
                        references.add(target)
                    elif target and target + ".zip" in tracked:
                        raise RuntimeError(f"Referenced archived asset requires extraction: {target}")
                    elif target and not (project / target).is_dir():
                        # Some upstream .meta files outlive their original asset.
                        unresolved.add(target)
        pending = references
    else:
        raise RuntimeError("Dependency traversal did not converge")
    report = {"source_revision": git(project, "rev-parse", "HEAD").strip(),
              "scene": SCENE, "referenced_assets": len(visited),
              "missing_upstream_assets": sorted(unresolved)}
    for name in restored_meta:
        if not (project / name[:-5]).exists():
            metadata = (project / name).resolve()
            if not metadata.is_relative_to(project / "Assets"):
                raise RuntimeError(f"Metadata cleanup escaped Assets: {metadata}")
            metadata.unlink()
    (project / "sketch-dependencies.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
