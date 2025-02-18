"""Pre-build steps for sphinx."""
import beamngpy  # needed for running sphinx-build because a later automatic import fails due to strange environment issue
import os
import subprocess
from shutil import copyfile, rmtree
from rosdoc2.verbs.build.generate_interface_docs import generate_interface_docs


def pre_build(root_dir, source_dir, output_dir):
    cleanup(root_dir, source_dir, output_dir)
    run_api_doc(root_dir, source_dir, output_dir)
    copy_files(root_dir, source_dir, output_dir)
    generate_msgs_interface(root_dir, source_dir, output_dir)


def run_api_doc(root_dir, source_dir, output_dir):
    src = f"{root_dir}/beamng_ros2/beamng_ros2"
    dst = f"{source_dir}/beamng_ros2/api"
    cmd = ["sphinx-apidoc", "-e", "-o", dst, src]
    result = subprocess.run(cmd, check=True, capture_output=True, text=True)
    print(result.stdout)
    print(result.stderr)


def copy_files(root_dir, source_dir, output_dir):
    files = [
        (f"{root_dir}/README.md", f"{source_dir}/README.md"),
        (f"{root_dir}/beamng_ros2/package.xml", f"{source_dir}/beamng_ros2/standard_docs/original/package.xml"),
        (f"{root_dir}/beamng_msgs/package.xml", f"{source_dir}/beamng_msgs/standard_docs/original/package.xml"),
    ]
    for src_file, dst_file in files:
        d = os.path.dirname(dst_file)
        if not os.path.isdir(d):
            os.makedirs(d)
        copyfile(src_file, dst_file)


def generate_msgs_interface(root_dir, source_dir, output_dir):
    path = f"{root_dir}/beamng_msgs"
    package = "beamng_msgs"
    output_dir_interfaces = f"{source_dir}/{package}/interfaces"
    if not os.path.isdir(output_dir_interfaces):
        os.makedirs(output_dir_interfaces)
    generate_interface_docs(path, package, output_dir_interfaces)


def cleanup(root_dir, source_dir, output_dir):
    items = [
        f"{source_dir}/README.md",
        f"{source_dir}/beamng_ros2/standard_docs/original",
        f"{source_dir}/beamng_msgs/standard_docs/original",
        f"{source_dir}/beamng_ros2/api",
        f"{source_dir}/beamng_msgs/interfaces",
    ]
    for item in items:
        if os.path.isfile(item):
            os.remove(item)
        elif os.path.isdir(item):
            rmtree(item)
