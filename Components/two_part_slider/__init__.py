"""Component Leg 2 joints 01 module"""

import pymel.core as pm
from pymel.core import datatypes

from mgear.shifter import component

from mgear.core import node, fcurve, applyop, vector, icon
from mgear.core import attribute, transform, primitive

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""
        self.root_guide = self.guide.pos["root"]

        """ Driver Objects """
        leg_root_t = transform.getTransformFromPos(self.root_guide)
        self.leg_root_offset = primitive.addTransform(self.root, self.getName("leg_root"), leg_root_t)

        leg_ankle_t = transform.getTransformFromPos(self.root_guide)
        self.leg_ankle_offset = primitive.addTransform(self.root, self.getName("ankle_root"), leg_ankle_t)

        leg_knee_t = transform.getTransformFromPos(self.root_guide)
        self.leg_knee_offset = primitive.addTransform(self.root, self.getName("knee_root"), leg_knee_t)


    def addAttributes(self):
        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(self.settings["ikrefarray"].split(","))
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam("ikref", "Ik Ref", 0, ref_names)

    def addOperators(self):
        # IK Handle Constraints
        pm.parentConstraint(self.leg_root_offset, self.leg_ankle_offset, self.leg_knee_offset, maintainOffset=True)

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        self.relatives["root"] = self.leg_knee_offset
        self.controlRelatives["root"] = self.leg_knee_offset
        # self.jointRelatives["root"] = 0

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
