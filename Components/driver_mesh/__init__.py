"""Component Leg 2 joints 01 module"""

import pymel.core as pm
from pymel.core import datatypes

from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, icon
from mgear.core import attribute, transform, primitive

import maya.OpenMaya as OpenMaya

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def __init__(self, rig, guide):
        super().__init__(rig, guide)
        self.rivets = []

    def proper_rivet(self, mesh_in: str, rivet_type="vtx", vtx=0, pos=[0, 0, 0], mesh_shape_in="", mesh_shape_orig_in="") -> [pm.nt.Transform, pm.nt.UvPin]:
        # im mad
        # normal cmds.Rivet() doesn't actually return the data it prints out, the uvPin and Rivet names.

        mesh_transform = pm.PyNode(mesh_in)

        if mesh_shape_in:
            mesh_shape = pm.PyNode(mesh_shape_in)  # Lets the user overwrite mesh_shape if wanted
        else:
            mesh_shape = mesh_transform.getShapes()[0]

        if mesh_shape_orig_in:
            mesh_shape_orig = pm.PyNode(mesh_shape_orig_in)  # Lets the user overwrite mesh_shape_orig if wanted
        else:
            # Check if orig shape exists, and if not, set the shape_orig to the first shape
            transform_shapes = mesh_transform.getShapes()
            does_orig_shape_exist = [i for i in transform_shapes if i.name().endswith("Orig")]
            if does_orig_shape_exist:
                mesh_shape_orig = does_orig_shape_exist[0]
            else:
                mesh_shape_orig = mesh_shape

        temp_loc = pm.spaceLocator()

        if rivet_type == "vtx":
            ws_vert_loc = pm.pointPosition(f"{mesh_transform}.vtx[{vtx}]", world=True)
        elif rivet_type == "pos":
            ws_vert_loc = pos

        temp_loc.t.set(ws_vert_loc)

        cpom_node = pm.createNode("closestPointOnMesh")
        temp_loc.t >> cpom_node.inPosition
        mesh_shape.outMesh >> cpom_node.inMesh

        u = cpom_node.parameterU.get()
        v = cpom_node.parameterV.get()

        pm.delete(temp_loc)
        pm.delete(cpom_node)

        rivet_loc = primitive.addLocator(parent=self.rivet_grp, name=f"{mesh_in.name()}_rivet_{vtx}")

        uv_pin_node = pm.createNode("uvPin")
        uv_pin_node.coordinate[0].coordinateU.set(u)
        uv_pin_node.coordinate[0].coordinateV.set(v)

        mesh_shape.outMesh >> uv_pin_node.deformedGeometry
        mesh_shape_orig.outMesh >> uv_pin_node.originalGeometry

        uv_pin_node.outputMatrix[0] >> rivet_loc.offsetParentMatrix

        return rivet_loc, uv_pin_node

    def rivets_per_face(self, object_to_rivet: str) -> pm.nt.Transform:
        rivet_object = pm.PyNode(object_to_rivet)

        for index, face in enumerate(rivet_object.faces):
            pt = face.__apimfn__().center(OpenMaya.MSpace.kWorld)
            rivet, _ = self.proper_rivet(mesh_in=object_to_rivet, rivet_type="pos", pos=pt, vtx=index)
            self.rivets.append(rivet)

    def name_arbitrary_mirrored_objects(self, group_name: str, centre_variance: int = 0.2) -> None:
        group = pm.PyNode(group_name)
        for object in group.getChildren():
            worldspace_translate = pm.xform(object, translation=True, worldSpace=True, query=True)
            lcr = "l" if worldspace_translate[0] >= centre_variance else "c"
            lcr = lcr if worldspace_translate[0] >= -centre_variance else "r"

            object.rename(f"{lcr}_{object.name()}")

        l_rvts = [i for i in group.getChildren() if i.name().startswith("l_")]
        r_rvts = [i for i in group.getChildren() if i.name().startswith("r_")]
        for r_object in r_rvts:
            worldspace_translate = pm.xform(r_object, translation=True, worldSpace=True, query=True)
            worldspace_translate_mirrored = [worldspace_translate[0] * -1, worldspace_translate[1], worldspace_translate[2]]

            for index, l_object in enumerate(l_rvts):
                l_worldspace_translation_dp = ["{:.2f}".format(i) for i in pm.xform(l_object, translation=True, worldSpace=True, query=True)]
                r_worldspace_translation_dp = ["{:.2f}".format(i) for i in worldspace_translate_mirrored]
                if l_worldspace_translation_dp == r_worldspace_translation_dp:
                    print(r_object, l_object)
                    r_object.rename(l_object.name().replace("l_", "r_"))
                    l_rvts.pop(index)
                    continue

        return None

    def addObjects(self):
        """Add all the objects needed to create the component."""
        self.root_guide = self.guide.pos["root"]
        self.driver_mesh = pm.PyNode("driver_mesh")

        t = transform.getTransformFromPos(self.root_guide)
        self.rivet_grp = primitive.addTransform(self.root, self.getName("rivets"), t)
        self.rivet_grp.hide()

        self.rivets_per_face(object_to_rivet=self.driver_mesh)

        self.name_arbitrary_mirrored_objects(group_name=self.rivet_grp.name())

        for rivet in self.rivets:
            self.jnt_pos.append({"obj": rivet, "name": f"{rivet.name()}"})

    def addAttributes(self):
        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(self.settings["ikrefarray"].split(","))
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam("ikref", "Ik Ref", 0, ref_names)

    def addOperators(self):
        # IK Handle Constraints
        pass

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        pass

    def addConnection(self):
        """Add more connection definition to the set"""
        self.connections["standard"] = self.connect_standard
        self.connections["orientation"] = self.connect_orientation

    def connect_standard(self):
        """standard connection definition for the component"""
        self.connectRef(self.settings["ikrefarray"], self.leg_ankle_offset)
        self.connect_standardWithSimpleIkRef()

    def connect_orientation(self):
        """Orient connection definition for the component"""
        self.connect_orientCns()
