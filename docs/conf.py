# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
from pathlib import Path


# -- Run pre-build steps -----------------------------------------------------
sys.path.append(str(Path(".").resolve()))

from pre_build import pre_build, cleanup

def setup(app):
    root_dir = str(Path(f"{app.srcdir}/..").resolve())
    pre_build(root_dir, app.srcdir, app.outdir)

    def build_finished(app, exception):
        if not exception:
            cleanup(root_dir, app.srcdir, app.outdir)

    app.connect('build-finished', build_finished)


# -- Project information -----------------------------------------------------

project = "BeamNG-ROS2"
copyright = "2024, BeamNG GmbH"
author = "BeamNG GmbH"


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.napoleon",
    "sphinx.ext.autodoc",
    "sphinx.ext.intersphinx",
    "sphinx.ext.extlinks",
    "myst_parser",
    "sphinx_multiversion",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

html_favicon = "_static/favicon.ico"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]


# source_suffix = '.rst'
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# -- Intersphinx options -----------------------------------------------------
intersphinx_mapping = {
    "beamngpy": ("https://documentation.beamng.com/api/beamngpy/", None),
    "rclpy": ("https://docs.ros.org/en/iron/p/rclpy/", None),
    "sensor_msgs": ("https://docs.ros.org/en/humble/p/sensor_msgs/", None),
    "std_msgs": ("https://docs.ros.org/en/humble/p/std_msgs/", None),
    "radar_msgs": ("https://index.ros.org/p/radar_msgs/", None),
    "viz_msgs": (
        "https://docs.ros.org/en/humble/p/visualization_msgs/",
        None,
    ),
    "geometry_msgs": ("https://docs.ros.org/en/humble/p/geometry_msgs/", None),
    "python": ("https://docs.python.org/3", None),
    "beamng_msgs": (
        "https://documentation.beamng.com/api/ros2/main/beamng_msgs/",
        None,
    ),
    # "numpy": ("https://numpy.org/doc/stable/", None),
}

# -- Napoleon options --------------------------------------------------------
napoleon_google_docstring = True

# -- Autodoc options ---------------------------------------------------------
autodoc_mock_imports = [
    "msgpack",
    "PIL",
    "matplotlib",
    "numpy",
    "seaborn",
    "sensor_msgs",
]
autodoc_typehints = "both"

# -- Extlinks options --------------------------------------------------------
extlinks = {
    "radar_msgs": (
        "https://docs.ros.org/en/noetic/api/radar_msgs/html/msg/%s.html",
        "%s",
    )
}

# -- Multi-version config ----------------------------------------------------
smv_tag_whitelist = r'^v.*$'
smv_branch_whitelist = r'^(main|dev)$'
