#!/usr/bin/env python3
"""Static checks over the tm12_bottle_sorting package.

These are here because each one corresponds to something that was actually
broken in this repo and cost time on the bench:

  * a launch file starting a node whose file had been renamed
  * nodes that roslaunch could not exec because the file was not +x or had
    no shebang
  * absolute paths into a developer's home directory

None of this needs ROS installed, so it runs on a plain runner in seconds.
"""

import os
import re
import sys
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(ROOT, "src", "tm12_bottle_sorting")

failures = []


def fail(msg):
    failures.append(msg)


def rel(path):
    return os.path.relpath(path, ROOT)


def cmake_targets():
    text = open(os.path.join(PKG, "CMakeLists.txt")).read()
    return set(re.findall(r"add_executable\(\s*(\S+)", text))


def check_launch_files():
    """Every <node type="..."> must name a script or a built executable."""
    targets = cmake_targets()
    launch_dir = os.path.join(PKG, "launch")

    for name in sorted(os.listdir(launch_dir)):
        if not name.endswith(".launch"):
            continue
        path = os.path.join(launch_dir, name)

        try:
            tree = ET.parse(path)
        except ET.ParseError as exc:
            fail("{}: not valid XML - {}".format(rel(path), exc))
            continue

        for node in tree.iter("node"):
            pkg = node.get("pkg")
            node_type = node.get("type")
            if pkg != "tm12_bottle_sorting" or not node_type:
                continue

            if node_type.endswith(".py"):
                script = os.path.join(PKG, "scripts", node_type)
                if not os.path.isfile(script):
                    fail("{}: node '{}' starts {}, which does not exist in "
                         "scripts/".format(rel(path), node.get("name"), node_type))
            elif node_type not in targets:
                fail("{}: node '{}' starts executable '{}', which no "
                     "add_executable() builds".format(
                         rel(path), node.get("name"), node_type))


def check_scripts_executable():
    """roslaunch execs scripts directly, so they need +x and a shebang."""
    scripts_dir = os.path.join(PKG, "scripts")
    for name in sorted(os.listdir(scripts_dir)):
        if not name.endswith(".py"):
            continue
        path = os.path.join(scripts_dir, name)

        if not os.access(path, os.X_OK):
            fail("{}: not executable (chmod +x)".format(rel(path)))

        with open(path) as handle:
            first = handle.readline()
        if not first.startswith("#!"):
            fail("{}: no shebang".format(rel(path)))
        elif "python3" not in first:
            fail("{}: shebang is '{}', expected python3".format(
                rel(path), first.strip()))


def check_no_absolute_paths():
    """Paths into someone's home directory do not survive a clone."""
    pattern = re.compile(r"/home/[a-z][a-z0-9_-]*/")
    for dirpath, dirnames, filenames in os.walk(PKG):
        dirnames[:] = [d for d in dirnames if d not in (".git", "runtime")]
        for name in filenames:
            if not name.endswith((".py", ".cpp", ".h", ".launch", ".txt", ".xml")):
                continue
            path = os.path.join(dirpath, name)
            with open(path, errors="replace") as handle:
                for lineno, line in enumerate(handle, 1):
                    if pattern.search(line):
                        fail("{}:{}: hardcoded home directory - resolve it "
                             "with ros::package::getPath or rospkg".format(
                                 rel(path), lineno))


def check_package_xml():
    path = os.path.join(PKG, "package.xml")
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        fail("{}: not valid XML - {}".format(rel(path), exc))
        return

    for field in ("name", "version", "description", "license"):
        if root.find(field) is None or not (root.find(field).text or "").strip():
            fail("{}: <{}> is missing or empty".format(rel(path), field))

    maintainer = root.find("maintainer")
    if maintainer is None:
        fail("{}: no <maintainer>".format(rel(path)))
    elif "todo" in (maintainer.get("email") or "").lower():
        fail("{}: maintainer email is still the catkin placeholder".format(rel(path)))


def check_submodules():
    """A gitlink with no .gitmodules entry clones as an empty directory."""
    gitmodules = os.path.join(ROOT, ".gitmodules")
    declared = set()
    if os.path.isfile(gitmodules):
        declared = set(re.findall(r"path\s*=\s*(\S+)", open(gitmodules).read()))

    for name in ("src/tmr_ros1", "src/yolov5"):
        if name not in declared:
            fail(".gitmodules: no entry for {}, so it clones empty".format(name))


for check in (check_launch_files, check_scripts_executable,
              check_no_absolute_paths, check_package_xml, check_submodules):
    check()

if failures:
    print("workspace checks failed:\n")
    for item in failures:
        print("  - {}".format(item))
    sys.exit(1)

print("workspace checks passed")
