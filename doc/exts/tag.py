#!/usr/bin/env python
# -*- coding: utf-8 -*-

from docutils import nodes

def tag_role(role, rawtext, text, lineno, inliner,
                       options={}, content=[]):

    categories = {'pymorse': 'middleware',
                  'ros': 'middleware',
                  'pocolibs': 'middleware',
                  'sockets': 'middleware',
                  'yarp': 'middleware',
                  'builder': 'api',
                  'datastream': 'access',
                  'service': 'access'}
    if text not in categories:
        msg = inliner.reporter.error(
            'Unknown category for tag "%s". Check exts/tag.py '
            'for available categories' % text, line=lineno)
        prb = inliner.problematic(rawtext, rawtext, msg)
        return [prb], [msg]
    
    node = nodes.literal(rawtext, text)
    node['classes'].append("tag")
    node['classes'].append("tag-" + categories[text])

    return [node], []


def setup(app):
    """Install the plugin.
    
    :param app: Sphinx application context.
    """
    app.add_role('tag', tag_role)
    return
