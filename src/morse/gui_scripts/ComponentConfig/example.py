vbl_info = {
    'name': 'Rickylbneder',
    'author': 'Rickylbneder',
    'version': (0, 4),
    'blender': (2, 5, 5),
    'api': 34513,
    'location': 'Select a Mesh: Tool Shelf > AnimAll panel',
    'description': 'Allows animation of mesh and lattice data (Shape Keys, VCols, VGroups, UVs)',
    'warning': '',
    'wiki_url': 'http://wiki.blender.org/index.php/Extensions:2.5/Py/Scripts/Animation/AnimAll',
    'tracker_url': 'http://projects.blender.org/tracker/index.php?'\
        'func=detail&aid=24874',
    'category': 'Animation'}
 
import bpy
class MyItem(bpy.types.IDPropertyGroup):
    mystring= bpy.props.StringProperty(name="Some title")
    mybool= bpy.props.BoolProperty(name="Some boolean",default= False)
    pass

#bpy.utils.register_class(MyItem)
 
#bpy.types.Object.myCollection= bpy.props.CollectionProperty(type= "MyItem")
bpy.types.Object.myCollection_index= bpy.props.IntProperty(
    min= -1,
    default= -1
)

bpy.types.Object.myCollection = bpy.props.CollectionProperty(type=MyItem)
#bpy.types.Object.myCollection = bpy.props.CollectionProperty(type = PropertyGroup)
#bpy.types.Object.myCollection_index = bpy.props.IntProperty(min = -1, default = -1)


bpy.types.Object.mychosenObject= bpy.props.StringProperty(name="Item")

## create operator to add or remove entries to/from  the Collection
class OBJECT_OT_add_remove_Collection_Items(bpy.types.Operator):
    bl_label = "Add or Remove"
    bl_idname = "collection.add_remove"
    add = bpy.props.BoolProperty(default = True)
    def invoke(self, context, event):
        add = self.add
        obj = context.object
        collection = obj.myCollection
        if add:
            # This add at the end of the collection list
            collection.add()
            collection[-1].name= "Item"
        else:
            # This remove on item in the collection list function of index value
            index = obj.myCollection_index
            collection.remove(index)
        return {'FINISHED'}
 
class OBJECT_PT_ObjectSelecting(bpy.types.Panel):
    bl_label = "Object Selecting"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "object"
    def draw(self, context):
        obj = context.object
        layout = self.layout
        ##show collection in Panel:
        row = layout.row()
        row.template_list(obj, "myCollection", obj, "myCollection_index")          # This show list for the collection
        ##show add/remove Operator
        col = row.column(align=True)
        col.operator("collection.add_remove", icon="ZOOMIN", text="")              # This show a plus sign button
        col.operator("collection.add_remove", icon="ZOOMOUT", text="").add = False # This show a minus sign button
        ##change name of Entry:
        if obj.myCollection:
            entry = obj.myCollection[obj.myCollection_index]
            layout.prop(entry, "name")
            ##show self created properties of myCollection
            layout.prop(entry, "mystring")
            layout.prop(entry, "mybool")
            
            layout.separator()
            ### search prop to search in myCollection:
            layout.prop_search(obj, "mychosenObject",    obj, "myCollection")
#bpy.utils.register_class(OBJECT_PT_ObjectSelecting)
#bpy.utils.register_class(OBJECT_OT_add_remove_Collection_Items)
#bpy.utils.register_class(MyItem)
#bpy.utils.register_class(OBJECT_OT_add_remove_Collection_Items)
