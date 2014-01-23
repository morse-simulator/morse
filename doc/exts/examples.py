from sphinx.directives import CodeBlock

def nop():
    pass

class Example(CodeBlock):
    required_arguments = 0
    def __init__(self, *args, **kwargs):
        CodeBlock.__init__(self, *args, **kwargs)
        self.arguments = ["python"]


def setup(app):
    """Install the plugin.
    
    :param app: Sphinx application context.
    """
    app.add_role('noautoexample', nop )
    app.add_directive('example', Example)
    return
