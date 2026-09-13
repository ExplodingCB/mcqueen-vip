"""ROS 2 message definition text for reading and writing MCAP logs without ROS.

rosbag2 stores each schema as the message definition followed by every
dependency, separated by a line of 80 '=' and a ``MSG: pkg/Type`` header.
``mcap_ros2`` parses that text, so a log written here reads back with the
same decoder the kart's logs use. The mcq_msgs definitions are assembled from
the ``.msg`` files in ``src/mcq_msgs`` so they never drift from the package.
"""

from __future__ import annotations

import re

from mcq_training import REPO

MSG_DIR = REPO / "src" / "mcq_msgs" / "msg"
SEPARATOR = "=" * 80

BUILTIN: dict[str, str] = {
    "builtin_interfaces/Time": "int32 sec\nuint32 nanosec\n",
    "std_msgs/Header": "builtin_interfaces/Time stamp\nstring frame_id\n",
    "geometry_msgs/Point": "float64 x\nfloat64 y\nfloat64 z\n",
    "geometry_msgs/Quaternion": "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n",
    "geometry_msgs/Pose": "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\n",
    "geometry_msgs/Vector3": "float64 x\nfloat64 y\nfloat64 z\n",
    "sensor_msgs/CompressedImage": "std_msgs/Header header\nstring format\nuint8[] data\n",
    "sensor_msgs/Image": (
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\n"
        "uint8 is_bigendian\nuint32 step\nuint8[] data\n"
    ),
    "sensor_msgs/CameraInfo": (
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring distortion_model\nfloat64[] d\n"
        "float64[9] k\nfloat64[9] r\nfloat64[12] p\nuint32 binning_x\nuint32 binning_y\n"
        "sensor_msgs/RegionOfInterest roi\n"
    ),
    "sensor_msgs/RegionOfInterest": "uint32 x_offset\nuint32 y_offset\nuint32 height\nuint32 width\nbool do_rectify\n",
}

_TYPE = re.compile(r"^([A-Za-z_][\w/]*?)(?:<=\d+)?(?:\[[^\]]*\])?$")  # base type, bounded string, array
_PRIMITIVES = frozenset(
    "bool byte char float32 float64 int8 uint8 int16 uint16 int32 uint32 int64 uint64 string wstring".split()
)


def raw_definition(name: str) -> str:
    """The bare text of ``pkg/Type``: builtin table first, then mcq_msgs/*.msg."""
    if name in BUILTIN:
        return BUILTIN[name]
    pkg, typ = name.split("/")
    if pkg != "mcq_msgs":
        raise KeyError(f"no definition for {name}")
    path = MSG_DIR / f"{typ}.msg"
    if not path.exists():
        raise KeyError(f"no definition for {name} ({path})")
    return path.read_text()


def dependencies(name: str, text: str) -> list[str]:
    """Message types referenced by the fields of ``name``'s definition, in order.
    Constants have primitive types and drop out with the primitives."""
    pkg = name.split("/")[0]
    deps: list[str] = []
    for raw in text.splitlines():
        parts = raw.split("#", 1)[0].split()
        if len(parts) < 2:
            continue
        base = _TYPE.match(parts[0])
        if base is None:
            continue
        t = base.group(1)
        if t in _PRIMITIVES:
            continue
        full = "std_msgs/Header" if t == "Header" else (t if "/" in t else f"{pkg}/{t}")
        if full not in deps:
            deps.append(full)
    return deps


def definition(name: str) -> str:
    """Concatenated definition text for ``pkg/Type`` in rosbag2's format."""
    main = raw_definition(name)
    seen: list[str] = []
    stack = dependencies(name, main)
    while stack:
        dep = stack.pop(0)
        if dep in seen:
            continue
        seen.append(dep)
        stack.extend(d for d in dependencies(dep, raw_definition(dep)) if d not in seen)
    parts = [main.rstrip("\n") + "\n"]
    for dep in seen:
        parts.append(f"{SEPARATOR}\nMSG: {dep}\n{raw_definition(dep).rstrip(chr(10))}\n")
    return "".join(parts)
