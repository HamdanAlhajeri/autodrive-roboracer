project = "AutoDRIVE RoboRacer"
author = "Team"
release = "0.1.0"

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
    "sphinx.ext.mathjax",
]

source_suffix = {".rst": "restructuredtext", ".md": "markdown"}
master_doc = "index"

html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "navigation_depth": 4,
    "titles_only": False,
}
html_static_path = ["_static"]
html_css_files = ["custom.css"]

exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]
