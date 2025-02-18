import os
from rosdoc2.verbs.build.generate_interface_docs import generate_interface_docs


def main():
    path = "beamng_msgs"
    package = "beamng_msgs"
    output_dir = "docs_build/beamng_msgs/interfaces"
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    generate_interface_docs(path, package, output_dir)


main()
