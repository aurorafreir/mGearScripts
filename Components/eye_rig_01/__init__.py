"""Eye Rig Component"""

from mgear.shifter import component
from mgear.core import attribute, transform, primitive, applyop
from pymel.core import datatypes
import pymel.core as pm
import maya.cmds as cmds

#############################################
# COMPONENT
#############################################

# TODO Remap eye follow so it's more intense to start and then tapers off
# TODO Split up vertical and horizontal follow
# TODO Add an eye blink
# TODO Add a main "Over" joint to hold the 3 skin joints

class Component(component.Main):
    """Shifter component Class"""

    def normalized_invert_floatmath_node(self, side:str, prefix:str, input_node:str, input_att:str) -> str:
        inv_node = pm.createNode("floatMath", name=f"eyelid_{prefix}_{side}_invert")
        pm.connectAttr(f"{input_node}.{input_att}", inv_node.floatB)
        inv_node.operation.set(1)
        inv_node_name = inv_node.name()

        return inv_node_name

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""

        self.ctl_size = self.settings["ctlSize"]

        self.comp_name = self.root.name().split("|")[-1]

        self.root_guide = self.guide.pos["root"]
        self.aim_guide = self.guide.pos["aim"]

        self.upper_full_rot = pm.dt.degrees(pm.dt.EulerRotation(self.guide.tra["upper_full"].rotate))
        self.lower_full_rot = pm.dt.degrees(pm.dt.EulerRotation(self.guide.tra["lower_full"].rotate))

        self.eye_upper_name = self.getName("eye_upper")
        self.eye_lower_name = self.getName("eye_lower")

        self.attr_settings = {"keyable": True, "hidden": False,
                              "hasMinValue": True, "hasMaxValue": True,
                              "minValue": 0, "maxValue": 1}

        """ Ctl creation """
        # MAIN SETUP
        eye_off_t = transform.getTransformFromPos(self.root_guide)
        self.eye_offset = primitive.addTransform(self.root, self.getName("eye_main_off"), eye_off_t)
        self.eye_ik_cns = primitive.addTransform(self.eye_offset, self.getName("eye_main_ik_cns"), eye_off_t)

        self.eye_ctl = self.addCtl(
            parent=self.eye_ik_cns,
            name="eye_ctl",
            m=eye_off_t,
            color=self.color_ik,
            iconShape="sphere",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        # AIM SETUP
        eye_aim_off_t = transform.getTransformFromPos(self.aim_guide)
        self.eye_aim_offset = primitive.addTransform(self.root, self.getName("eye_main_aim_off"), eye_aim_off_t)

        self.eye_aim_ctl = self.addCtl(
            parent=self.eye_aim_offset,
            name="eye_aim_ctl",
            m=eye_aim_off_t,
            color=self.color_ik,
            iconShape="circle",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        eye_aim_wuo_off_t = transform.getTransformFromPos(self.root_guide)
        self.eye_aim_wuo_grp = primitive.addTransform(self.root, self.getName("eye_main_aim_wuo_grp"), eye_aim_wuo_off_t)

        # EYELIDS
        self.ul_off = 0.3
        # UPPER SETUP
        eye_upper_off_t = transform.getTransformFromPos(self.root_guide)+[0,0,0,0, 0,0,0,0, 0,0,0,0, 0,self.ul_off,0,0]
        self.eye_upper_offset = primitive.addTransform(self.root, self.getName("eye_main_upper_off"), eye_upper_off_t)

        self.eye_upper_ctl = self.addCtl(
            parent=self.eye_upper_offset,
            name="eye_upper_ctl",
            m=eye_upper_off_t,
            color=self.color_ik,
            iconShape="null",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        # LOWER SETUP
        eye_lower_off_t = transform.getTransformFromPos(self.root_guide)+[0,0,0,0, 0,0,0,0, 0,0,0,0, 0,-self.ul_off,0,0]
        self.eye_lower_offset = primitive.addTransform(self.root, self.getName("eye_main_lower_off"), eye_lower_off_t)

        self.eye_lower_ctl = self.addCtl(
            parent=self.eye_lower_offset,
            name="eye_lower_ctl",
            m=eye_lower_off_t,
            color=self.color_ik,
            iconShape="null",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        """ Driver Joints """
        driver_joints_vis = False

        self.eye_joint = primitive.addJoint(
            self.root,
            self.getName("eye_driver"),
            eye_off_t,
            driver_joints_vis,
        )
        self.eye_upper_joint = primitive.addJoint(
            self.root,
            self.getName("eye_upper_driver"),
            eye_upper_off_t,
            driver_joints_vis,
        )
        self.eye_lower_joint = primitive.addJoint(
            self.root,
            self.getName("eye_lower_driver"),
            eye_lower_off_t,
            driver_joints_vis,
        )

        """ Attribute Setup """
        for attr_name in ["Follow", "Lower", "Upper"]:
            pm.addAttr(self.eye_aim_ctl, longName=f"Eyelid_{attr_name}", **self.attr_settings)

        self.eye_aim_ctl.attr(f"Eyelid_Follow").set(0.15)

        """ Upper and Lower Eyelid Functionality """
        for u_l in ["Lower", "Upper"]:
            # EYELID FOLLOW SETUP
            eye_follow_invfm = self.normalized_invert_floatmath_node(side=self.side, prefix=f"{u_l}_Follow",
                                                                     input_node=self.eye_aim_ctl,
                                                                     input_att=f"Eyelid_Follow")
            eye_follow_invfm = pm.PyNode(eye_follow_invfm)

            eye_follow_abnar = pm.createNode("animBlendNodeAdditiveRotation", name=f"{self.comp_name}_eyelid_follow_abnar")

            # pm.connectAttr(f"{eye_ctl.name()}.{side}_Eyelid_Follow", eye_follow_invfm.floatB)
            eye_follow_invfm.outFloat >> eye_follow_abnar.weightA
            self.eye_aim_ctl.Eyelid_Follow >> eye_follow_abnar.weightB

            self.eye_joint.rotate >> eye_follow_abnar.inputB

            # EYELID CLOSE SETUP
            eye_close_invfm = self.normalized_invert_floatmath_node(side=self.side, prefix=f"{u_l}_Close",
                                                                    input_node=self.eye_aim_ctl,
                                                                    input_att=f"Eyelid_{u_l}")
            eye_close_invfm = pm.PyNode(eye_close_invfm)

            eye_close_abnar = pm.createNode("animBlendNodeAdditiveRotation", name=f"{self.comp_name}_eyelid_close_abnar")

            eye_close_invfm.outFloat >> eye_close_abnar.weightA
            self.eye_aim_ctl.attr(f"Eyelid_{u_l}") >> eye_close_abnar.weightB

            eye_close_interactive_loc = pm.spaceLocator()
            eye_close_interactive_loc.rename(f"Eyelid_{u_l}_Closed_DELETEME_loc")

            eye_close_interactive_loc.t.set(eye_off_t.translate)
            pm.parent(eye_close_interactive_loc, self.root)
            eye_close_interactive_loc.r >> eye_close_abnar.inputB

            # EYELID BLEND SETUP
            eye_blend_abnar = pm.createNode("animBlendNodeAdditiveRotation", name=f"eyelid_blend_{self.comp_name}_abnar")
            eye_follow_abnar.output >> eye_blend_abnar.inputA
            eye_close_abnar.output >> eye_blend_abnar.inputB

            u_l_pm = self.eye_upper_joint
            u_l_rot = self.upper_full_rot
            if u_l == "Lower":
                u_l_pm = self.eye_lower_joint
                u_l_rot = self.lower_full_rot

            eye_close_interactive_loc.r.set(u_l_rot)

            pm.connectAttr(eye_blend_abnar.output, u_l_pm.rotate)

        """ Skin Joints """
        self.jnt_pos.append([self.eye_joint, "eye", False])
        self.jnt_pos.append([self.eye_upper_joint, "eye_upper", "eye_main"])
        self.jnt_pos.append([self.eye_lower_joint, "eye_lower", "eye_main"])

    def addAttributes(self):
        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(self.settings["ikrefarray"].split(","))
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam("ikref", "Ik Ref", 0, ref_names)

    def addOperators(self):
        # IK Handle Constraints
        # """ Aim Constraint """
        pm.aimConstraint(self.eye_aim_ctl, self.eye_ik_cns,
                         maintainOffset=True,
                         worldUpType="objectrotation",
                         wuo=self.eye_aim_wuo_grp
                         )

        applyop.gear_matrix_cns(self.eye_ctl, self.eye_joint)


    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        self.aliasRelatives["root"] = "ctl"

        self.relatives["root"] = self.eye_aim_offset
        self.controlRelatives["root"] = self.eye_aim_offset

        self.jointRelatives["eye_main"] = 0

    def addConnection(self):
        """Add more connection definition to the set"""
        self.connections["standard"] = self.connect_standard
        self.connections["orientation"] = self.connect_orientation

    def connect_standard(self):
        """standard connection definition for the component"""
        self.connectRef(self.settings["ikrefarray"], self.eye_aim_offset)
        self.connect_standardWithSimpleIkRef()

    def connect_orientation(self):
        """Orient connection definition for the component"""
        self.connect_orientCns()
