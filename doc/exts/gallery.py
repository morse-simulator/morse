#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import glob
from docutils import nodes
from docutils.parsers.rst import directives
from sphinx.util.compat import Directive

MEDIA_PATH="../doc/media/"
IMAGE_PATH="_images/"
HTML_PATH="user/"
RST_PATH="morse/" + HTML_PATH

def css(d):
    return "; ".join(sorted("%s: %s" % kv for kv in d.items()))

class gallery(nodes.General, nodes.Element): pass

def visit_gallery_node(self, node):

    images_per_row = 4

    style = {
        "position": "relative",
    }
    self.body.append(self.starttag(node, "table", style=css(style)))
    images =  os.listdir(os.path.join(MEDIA_PATH, node["directory"]))
    for i in range(len(images))[::images_per_row]:
        self.body.append("<tr>")
        for j in range(images_per_row):
            if i + j < len(images):
                obj = images[i + j][:-4] #remove extension
                self.body.append("<td style='text-align:center'><a href='" + HTML_PATH + node["directory"] + "/" + obj + ".html'><img style='width: 200px;' src='" + IMAGE_PATH + obj + ".png' /><br/><em>" + obj + "</em></a></td>")
        self.body.append("</tr>")
    self.body.append("</table>")

def depart_gallery_node(self, node):
    pass

class Gallery(Directive):
    has_content = True
    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = False

    def run(self):
        return [gallery(directory=self.arguments[0])]

def setup(app):
    app.add_node(gallery, html=(visit_gallery_node, depart_gallery_node))
    app.add_directive("gallery", Gallery)
