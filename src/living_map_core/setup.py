from setuptools import setup
import glob
pkg = "living_map_core"
setup(
    name=pkg, version="4.0.0", packages=[pkg],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + pkg]),
                ("share/" + pkg, ["package.xml"]),
                ("share/" + pkg + "/launch", glob.glob("launch/*.py")),
                ("share/" + pkg + "/rviz", glob.glob("rviz/*.rviz")),
                ("share/" + pkg + "/worlds", glob.glob("worlds/*.sdf")),
                ("share/" + pkg + "/maps", glob.glob("maps/*.json")),
                ("share/" + pkg + "/scripts", glob.glob("scripts/*.sh"))],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="team", maintainer_email="team@example.com",
    description="The Living Map: spatial memory for emergency robots (IEEE TSYP14)", license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "writer_node = living_map_core.nodes:main_writer",
        "mesh_node = living_map_core.nodes:main_mesh",
        "gateway_node = living_map_core.nodes:main_gateway",
        "command_post_node = living_map_core.nodes:main_command_post",
        "ambulance_node = living_map_core.nodes:main_ambulance",
        "firetruck_node = living_map_core.nodes:main_firetruck",
        "drone_node = living_map_core.nodes:main_drone",
        "sim_view_node = living_map_core.nodes:main_sim_view",
        "gz_bridge_node = living_map_core.nodes:main_gz_bridge",
        "order = living_map_core.order:main",
        "standalone_demo = living_map_core.standalone:main",
        "airgap_auditor = living_map_core.audit:main",
        "offline_demo = living_map_core.offline:main_demo",
        "fault_tests = living_map_core.offline:main_faults",
    ]},
)
