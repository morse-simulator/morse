#!/usr/bin/env python
# -*- coding: utf-8 -*-

from docutils import nodes


def tag_role(role, rawtext, text, lineno, inliner,
                       options={}, content=[]):

    categories = {'pymorse': ('middleware', 'pymorse'),
                  'ros': ('middleware', 'user/middlewares/ros'),
                  'pocolibs': ('middleware', 'user/middlewares/pocolibs'),
                  'yarp_json': ('middleware', 'user/middlewares/yarp'),
                  'text': ('middleware', 'user/middlewares/text'),
                  'socket': ('middleware', 'user/middlewares/socket'),
                  'yarp': ('middleware', 'user/middlewares/yarp'),
                  'moos': ('middleware', 'user/middlewares/moos'),
                  'builder': ('api', 'user/builder'),
                  'datastream': ('access', None),
                  'service': ('access', 'dev/services')}
    if text not in categories:
        msg = inliner.reporter.error(
            'Unknown category for tag "%s". Check exts/tag.py '
            'for available categories' % text, line=lineno)
        prb = inliner.problematic(rawtext, rawtext, msg)
        return [prb], [msg]
    
    # deseperately tried to generate a relative path to the doc root
    # but it's a failure.
    # inliner.document.settings.env may be a good starting point
    
    #url = categories[text][1]
    #node = None
    #if url:
    #    import pdb;pdb.set_trace()
    #    node = nodes.reference(rawtext, text, refuri = url + ".html")
    #else:
    #    node = nodes.literal(rawtext, text)

    node = nodes.literal(rawtext, text)
    node['classes'].append("tag")
    node['classes'].append("tag-" + categories[text][0])

    return [node], []


def setup(app):
    """Install the plugin.
    
    :param app: Sphinx application context.
    """
    app.add_role('tag', tag_role)
    return
