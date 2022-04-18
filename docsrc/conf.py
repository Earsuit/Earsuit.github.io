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
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'XIANYU'
copyright = '2022, Earsuit'
author = 'Earsuit'

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
"ablog",
"sphinx.ext.intersphinx",
"sphinx_panels",
"sphinx_sitemap",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []

numfig = True
numfig_secnum_depth = 0
numfig_format = {'figure': 'Figure %s', 'table': 'Table %s', 'code-block': 'Code Block %s'}

math_number_all = True
math_eqref_format = '({number})'
math_numfig = True

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'pydata_sphinx_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

fontawesome_included = True

html_sidebars = {
    "index": ["aboutme.html", "archives.html"],
    "about": ["aboutme.html"],
    "blogs": ["tagcloud.html", "archives.html"],
    "blog/**": ["postcard.html", "archives.html"],
}

html_theme_options = {
    # If you want to configure Twitter or Github social media buttons to show up to the right of your nav bar,
    # you can use the "github_url" and "twitter_url" options:
    "github_url": "https://github.com/Earsuit/",
    # You can also change the text that is in the search bar before people click on it by setting the
    # "search_bar_text"
    "search_bar_text": "Search for treasure...",
    # By default your site will have a search bar in the nav bar, but when we include the about.html,
    # this gets removed to so you can add one to the top "navbar" instead
    "navbar_end": ["navbar-icon-links.html", "search-field.html"],
    # Hiding the previous and next buttons
    "show_prev_next": False,
}

def setup(app):
    app.add_css_file("custom.css")

   
# If you used the blog file path in part one you do not need to set this value, however if you change it,
# this should be set to the same value.
# blog_path = "posts"

# blog_feed_fulltext: Choose to display full text in blog feeds.
blog_feed_fulltext = True

# Glob pattern that grabs all posts so you don't need to specify which posts are blog posts in each post
# This pattern facilitates a folder structure such as posts/2020/my-awesome-post.rst
blog_post_pattern = "posts/*/*"

# post_redirect_refresh: Number of seconds (default is 5) that a redirect page waits before refreshing the page
# to redirect to the post.
post_redirect_refresh = 1

# post_auto_image: Index of the image that will be displayed in the excerpt of the post. Default is 0, meaning no
# image. Setting this to 1 will include the first image
post_auto_image = 0

# post_auto_excerpt: Number of paragraphs (default is 1) that will be displayed as an excerpt from the post. Setting
# this 0 will result in displaying no post excerpt in archive pages.
post_auto_excerpt = 1

# -- Options for site map output -------------------------------------------------

# Set the value of html_baseurl in your Sphinx conf.py to the current base URL of your documentation.
html_baseurl = 'https://earsuit.github.io/'

# Set sitemap_filename in conf.py to the desired filename
sitemap_filename = "sitemap.xml"

# Alternative languages are either manually set by sitemap_locales option or auto-detected by the extension 
# from the locale_dirs config value, so make sure one of those is set.
sitemap_locales = [None]