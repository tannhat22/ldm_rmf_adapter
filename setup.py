import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ldm_rmf_adapter"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tan Nhat",
    maintainer_email="nguyentannhat2298@gmail.com",
    description="VDM Client as Open-RMF Lift and Door Adapter",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ldm_rmf_adapter = ldm_rmf_adapter.ldm_rmf_adapter:main",
            "send_adapter_lift_requests = ldm_rmf_adapter.send_adapter_lift_requests:main",
        ],
    },
)
