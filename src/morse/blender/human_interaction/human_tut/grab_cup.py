from bge import logic

co = logic.getCurrentController()
ow = co.owner

scene = logic.getCurrentScene()
scriptHolder = scene.objects['TUT_Script_Holder']

def test():
    if ow.parent and scriptHolder['Level'] == 4:
        if not 'init' in ow:
            ow['init'] = True
            scriptHolder['Level'] = 5
        
            
