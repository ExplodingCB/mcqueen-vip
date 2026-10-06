from pathlib import Path

from setuptools import find_packages, setup


def asset_files(folder):
    return [str(path) for path in sorted(Path(folder).iterdir()) if path.is_file()]


setup(
    name="mcq_telemetry",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mcq_telemetry"]),
        ("share/mcq_telemetry", ["package.xml", "README.md"]),
        *[("share/mcq_telemetry/" + folder, asset_files(folder)) for folder in ("config", "launch", "layouts")],
    ],
    install_requires=["setuptools", "pyyaml"],
    tests_require=["pytest"],
    maintainer="Chase Culbertson",
    maintainer_email="culbertsonprime@gmail.com",
    description="Best-effort pit telemetry, Foxglove layouts and lap timing.",
    license="MIT",
    entry_points={"console_scripts": ["telemetry_node = mcq_telemetry.node:main"]},
)
