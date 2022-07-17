"""Digitigrade Leg Component"""

from mgear.shifter import component
from mgear.core import attribute, transform, primitive, applyop
from pymel.core import datatypes
import pymel.core as pm

#############################################
# COMPONENT
#############################################

# TODO: fix PV parenting
# TODO: create ankle lock
# TODO: create FK ankle w/ IK upperleg
# TODO: twist joints
# TODO: Setup full attr locking
# TODO: fix toe rotations in ik mode when moving heel ctl


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""

        # if self.settings["neutralRotation"]:
        #     t = transform.getTransformFromPos(self.guide.pos["root"])
        # else:
        #     t = self.guide.tra["root"]
        #     if self.settings["mirrorBehaviour"] and self.negate:
        #         scl = [1, 1, -1]
        #     else:
        #         scl = [1, 1, 1]
        #     t = transform.setMatrixScale(t, scl)
        # if self.settings["joint"] and self.settings["leafJoint"]:
        #     pm.displayInfo("Skipping ctl creation, just leaf joint")
        #     self.have_ctl = False
        # else:
        #     self.have_ctl = True
        #
        #     # we need to set the rotation order before lock any rotation axis
        #     if self.settings["k_ro"]:
        #         rotOderList = ["XYZ", "YZX", "ZXY", "XZY", "YXZ", "ZYX"]
        #
        #     params = [
        #         s for s in ["tx", "ty", "tz", "ro", "rx", "ry", "rz", "sx", "sy", "sz"] if self.settings["k_" + s]
        #     ]

        if self.side == "L" or self.side == "C":
            ctl_one_x_matrix = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1]
        else:
            # ctl_one_x_matrix = [-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1]  # this breaks it for some reason
            ctl_one_x_matrix = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1]

        self.ctl_size = 2 * self.settings["ctlSize"]


        ### IK CONTROLS
        t = transform.getTransformFromPos(self.guide.pos["hip"])
        self.ik_cns = primitive.addTransform(self.root, self.getName("hip_ik_cns"), t * ctl_one_x_matrix)

        self.ik_hip = self.addCtl(
            parent=self.ik_cns,
            name="hip_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        t = transform.getTransformFromPos(self.guide.pos["paw"])
        self.ik_cns_ss = primitive.addTransform(
            self.root, self.getName("paw_ik_cns_ss"), [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1]
        )
        self.ik_cns = primitive.addTransform(self.ik_cns_ss, self.getName("paw_ik_cns"), t * ctl_one_x_matrix)
        self.ik_paw = self.addCtl(
            parent=self.ik_cns,
            name="paw_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        t = transform.getTransformFromPos(self.guide.pos["toe"])
        self.ik_cns_heelpush_ss = primitive.addTransform(
            self.ik_paw, self.getName("paw_ik_heelpush_cns_ss"), [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1]
        )
        self.ik_heelpush_cns = primitive.addTransform(self.ik_cns_heelpush_ss, self.getName("paw_ik_heelpush_cns"), t * ctl_one_x_matrix)
        self.ik_heelpush = self.addCtl(
            parent=self.ik_heelpush_cns,
            name="paw_heelpush_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="diamond",
            w=self.ctl_size * 0.7,
            h=self.ctl_size * 0.7,
            d=self.ctl_size * 0.7,
        )

        t = transform.getTransformFromPos(self.guide.pos["paw"])
        self.ik_heelpush_pawloc = primitive.addTransform(self.ik_heelpush, self.getName("paw_ik_heelpush_cns"), t * ctl_one_x_matrix)

        t = transform.getTransformFromPos(self.guide.pos["paw"])
        self.ik_cns = primitive.addTransform(self.ik_heelpush, self.getName("paw_rotate_ik_cns"), t * ctl_one_x_matrix)
        toffset = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -2, 0]
        self.paw_rotator_npo = primitive.addTransform(
            self.ik_cns, self.getName("paw_rotate_ik_npo"), t * ctl_one_x_matrix + toffset
        )
        self.paw_rotator = self.addCtl(
            parent=self.paw_rotator_npo,
            name="paw_rotator_ctl",
            m=t * ctl_one_x_matrix + toffset,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size * 0.5,
            h=self.ctl_size * 0.5,
            d=self.ctl_size * 0.5,
        )

        # POLE VECTOR
        t1 = self.guide.pos["knee"]
        t2 = self.guide.pos["ankle"]
        pos = [(t1[0] + t2[0]) / 2, (t1[1] + t2[1]) / 2, t2[2] - (.5 * self.size)]
        t = transform.getTransformFromPos(pos)
        self.ik_cns = primitive.addTransform(self.root, self.getName("leg_pv_ik_cns"), t * ctl_one_x_matrix)
        self.ik_pv_npo = primitive.addTransform(self.ik_cns, self.getName("leg_pv_ik_npo"), t * ctl_one_x_matrix)
        self.ik_pv = self.addCtl(
            parent=self.ik_pv_npo,
            name="leg_pv",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="diamond",
            w=self.ctl_size * 0.5,
            h=self.ctl_size * 0.5,
            d=self.ctl_size * 0.5,
        )

        ### FK CONTROLS
        t = transform.getTransformFromPos(self.guide.pos["hip"])
        self.fk_cns = primitive.addTransform(self.root, self.getName("hip_fk_cns"), t * ctl_one_x_matrix)

        self.fk_hip = self.addCtl(
            parent=self.fk_cns,
            name="hip_fk_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        t = transform.getTransformFromPos(self.guide.pos["knee"])
        self.fk_cns = primitive.addTransform(self.fk_hip, self.getName("knee_fk_cns"), t * ctl_one_x_matrix)

        self.fk_knee = self.addCtl(
            parent=self.fk_cns,
            name="knee_fk_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        t = transform.getTransformFromPos(self.guide.pos["ankle"])
        self.fk_cns = primitive.addTransform(self.fk_knee, self.getName("ankle_fk_cns"), t * ctl_one_x_matrix)

        self.fk_ankle = self.addCtl(
            parent=self.fk_cns,
            name="ankle_fk_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        t = transform.getTransformFromPos(self.guide.pos["paw"])
        self.fk_cns = primitive.addTransform(self.fk_ankle, self.getName("paw_fk_cns"), t * ctl_one_x_matrix)

        self.fk_paw = self.addCtl(
            parent=self.fk_cns,
            name="paw_fk_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="cube",
            w=self.ctl_size,
            h=self.ctl_size,
            d=self.ctl_size,
        )

        ### JOINT SETUP
        ### IK DRIVER JOINTS
        driver_joints_vis = False
        self.hip_ik_joint = primitive.addJoint(
            self.root,
            self.getName("hip_ik_driver"),
            transform.getTransformFromPos(self.guide.pos["hip"]),
            driver_joints_vis,
        )
        self.knee_ik_joint = primitive.addJoint(
            self.hip_ik_joint,
            self.getName("knee_ik_driver"),
            transform.getTransformFromPos(self.guide.pos["knee"]),
            driver_joints_vis,
        )
        self.ankle_ik_joint = primitive.addJoint(
            self.knee_ik_joint,
            self.getName("ankle_ik_driver"),
            transform.getTransformFromPos(self.guide.pos["ankle"]),
            driver_joints_vis,
        )
        self.paw_ik_joint = primitive.addJoint(
            self.ankle_ik_joint,
            self.getName("paw_ik_driver"),
            transform.getTransformFromPos(self.guide.pos["paw"]),
            driver_joints_vis,
        )
        self.toe_ik_joint = primitive.addJoint(
            self.paw_ik_joint,
            self.getName("toe_ik_driver"),
            transform.getTransformFromPos(self.guide.pos["toe"]),
            driver_joints_vis,
        )

        # MAIN IK SOLVER JOINTS
        self.ik1_jnt = primitive.addJoint(
            self.root, self.getName("ik1"), transform.getTransformFromPos(self.guide.pos["hip"]), driver_joints_vis
        )
        pos = [(t1[0] + t2[0]) / 2, (t1[1] + t2[1]) / 2, t2[2] - 2]
        self.ik2_jnt = primitive.addJoint(
            self.ik1_jnt, self.getName("ik2"), transform.getTransformFromPos(pos), driver_joints_vis
        )
        self.ik3_jnt = primitive.addJoint(
            self.ik2_jnt, self.getName("ik3"), transform.getTransformFromPos(self.guide.pos["paw"]), driver_joints_vis
        )

        # REVERSE JOINTS
        self.rev1_jnt = primitive.addJoint(
            self.root, self.getName("rev1"), transform.getTransformFromPos(self.guide.pos["paw"]), driver_joints_vis
        )
        self.rev2_jnt = primitive.addJoint(
            self.rev1_jnt,
            self.getName("rev2"),
            transform.getTransformFromPos(self.guide.pos["ankle"]),
            driver_joints_vis,
        )

        ### FK JOINTS
        driver_joints_vis = False
        self.hip_fk_joint = primitive.addJoint(
            self.root,
            self.getName("hip_fk_driver"),
            transform.getTransformFromPos(self.guide.pos["hip"]),
            driver_joints_vis,
        )
        self.knee_fk_joint = primitive.addJoint(
            self.hip_fk_joint,
            self.getName("knee_fk_driver"),
            transform.getTransformFromPos(self.guide.pos["knee"]),
            driver_joints_vis,
        )
        self.ankle_fk_joint = primitive.addJoint(
            self.knee_fk_joint,
            self.getName("ankle_fk_driver"),
            transform.getTransformFromPos(self.guide.pos["ankle"]),
            driver_joints_vis,
        )
        self.paw_fk_joint = primitive.addJoint(
            self.ankle_fk_joint,
            self.getName("paw_fk_driver"),
            transform.getTransformFromPos(self.guide.pos["paw"]),
            driver_joints_vis,
        )
        self.toe_fk_joint = primitive.addJoint(
            self.paw_fk_joint,
            self.getName("toe_fk_driver"),
            transform.getTransformFromPos(self.guide.pos["toe"]),
            driver_joints_vis,
        )

        ### BLEND JOINTS
        self.hip_blend_joint = primitive.addJoint(
            self.root,
            self.getName("hip_blend_driver"),
            transform.getTransformFromPos(self.guide.pos["hip"]),
            driver_joints_vis,
        )
        self.knee_blend_joint = primitive.addJoint(
            self.hip_blend_joint,
            self.getName("knee_blend_driver"),
            transform.getTransformFromPos(self.guide.pos["knee"]),
            driver_joints_vis,
        )
        self.ankle_blend_joint = primitive.addJoint(
            self.knee_blend_joint,
            self.getName("ankle_blend_driver"),
            transform.getTransformFromPos(self.guide.pos["ankle"]),
            driver_joints_vis,
        )
        self.paw_blend_joint = primitive.addJoint(
            self.ankle_blend_joint,
            self.getName("paw_blend_driver"),
            transform.getTransformFromPos(self.guide.pos["paw"]),
            driver_joints_vis,
        )
        self.toe_blend_joint = primitive.addJoint(
            self.paw_blend_joint,
            self.getName("toe_blend_driver"),
            transform.getTransformFromPos(self.guide.pos["toe"]),
            driver_joints_vis,
        )

        self.toe_blend_end = primitive.addTransform(
            self.root,
            self.getName("toe_blend_end"),
            transform.getTransformFromPos(self.guide.pos["toe"]),
        )

        ### ADDITIONAL CONTROLS
        #
        t = transform.getTransformFromPos(self.guide.pos["toe"])
        self.toe_cns = primitive.addTransform(self.root, self.getName("toe_cns"), t * ctl_one_x_matrix)

        self.toe = self.addCtl(
            parent=self.toe_cns,
            name="toe_ctl",
            m=t * ctl_one_x_matrix,
            color=self.color_ik,
            iconShape="diamond",
            w=self.ctl_size * .5,
            h=self.ctl_size * .5,
            d=self.ctl_size * .5,
        )

        ### SKIN JOINTS
        self.jnt_pos.append([self.hip_blend_joint, "hip", False])
        self.jnt_pos.append([self.knee_blend_joint, "knee", False])
        self.jnt_pos.append([self.ankle_blend_joint, "ankle", False])
        self.jnt_pos.append([self.paw_blend_joint, "paw", False])
        self.jnt_pos.append([self.toe, "toe", False])

        ### SOLVER SETUP
        self.ik_ikh = primitive.addIkHandle(
            self.root, self.getName("ik_ikh"), [self.ik1_jnt, self.ik3_jnt], "ikRPsolver"
        )
        self.upleg_ikh = primitive.addIkHandle(
            self.root, self.getName("upleg_ikh"), [self.hip_ik_joint, self.ankle_ik_joint], "ikRPsolver"
        )
        self.downleg_ikh = primitive.addIkHandle(
            self.root, self.getName("downleg_ikh"), [self.ankle_ik_joint, self.paw_ik_joint], "ikSCsolver"
        )

    def addAttributes(self):
        # Ref
        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(self.settings["ikrefarray"].split(","))
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam("ikref", "Ik Ref", 0, ref_names)

        # Anim -------------------------------------------
        self.blend_att = self.addAnimParam("blend",
                                           "Fk/Ik Blend",
                                           "double",
                                           1,
                                           0,
                                           1)

    def addOperators(self):
        # IK Handle Constraints
        applyop.gear_matrix_cns(self.paw_rotator, self.ik_ikh)
        applyop.gear_matrix_cns(self.ik_heelpush_pawloc, self.downleg_ikh)
        applyop.gear_matrix_cns(self.rev2_jnt, self.upleg_ikh)
        applyop.gear_matrix_cns(self.ik_hip, self.hip_ik_joint)

        applyop.oriCns(self.ik3_jnt, self.paw_rotator_npo, maintainOffset=True)

        # Orient Constrain paw rotator control to reverse joint 1
        pm.pointConstraint(self.ik_heelpush_pawloc, self.rev1_jnt, maintainOffset=True)
        applyop.oriCns(self.paw_rotator, self.rev1_jnt, maintainOffset=True)
        # Orient Constrain paw control to paw jnt
        applyop.oriCns(self.ik_heelpush, self.paw_ik_joint, maintainOffset=True)

        # applyop.gear_matrix_cns("global_C0_ctl", self.ik_cns_ss)
        pm.parentConstraint("global_C0_ctl", pm.PyNode(self.ik_cns_ss), maintainOffset=True)

        pm.poleVectorConstraint(pm.PyNode(self.ik_pv), self.ik_ikh)
        pm.poleVectorConstraint(pm.PyNode(self.ik_pv), self.upleg_ikh)

        pm.PyNode(self.upleg_ikh).twist.set(180)

        applyop.gear_matrix_cns(self.fk_hip, self.hip_fk_joint)
        applyop.gear_matrix_cns(self.fk_knee, self.knee_fk_joint)
        applyop.gear_matrix_cns(self.fk_ankle, self.ankle_fk_joint)
        applyop.gear_matrix_cns(self.fk_paw, self.paw_fk_joint)

        # TODO: This needs to be fixed to use an intermediary joint for rotation rather than the ik_paw ctl
        #   - Currently the toe breaks in FK mode as it only uses the IK ctl rotation
        pm.pointConstraint(self.toe_blend_joint, self.toe_cns)
        pm.orientConstraint(self.ik_paw, self.toe_cns)

        # Driver and Blend joint constraints
        self.hip_blend_cns = pm.orientConstraint(self.hip_ik_joint, self.hip_fk_joint, self.hip_blend_joint)
        self.knee_blend_cns = pm.orientConstraint(self.knee_ik_joint, self.knee_fk_joint, self.knee_blend_joint)
        self.ankle_blend_cns = pm.orientConstraint(self.ankle_ik_joint, self.ankle_fk_joint, self.ankle_blend_joint)
        self.paw_blend_cns = pm.orientConstraint(self.paw_ik_joint, self.paw_fk_joint, self.paw_blend_joint)
        self.toe_blend_cns = pm.orientConstraint(self.toe_ik_joint, self.toe_fk_joint, self.toe_blend_joint)

        self.flip_node = pm.createNode("floatMath")
        self.flip_node.operation.set(1)

        #
        applyop.gear_matrix_cns(self.toe_blend_joint, self.toe_blend_end)

        # Attr locking
        pv_pm = pm.PyNode(self.ik_pv)
        paw_pm = pm.PyNode(self.ik_paw)
        paw_rot_pm = pm.PyNode(self.paw_rotator)
        hip_pm = pm.PyNode(self.ik_hip)

        for ctl_attr in [
            paw_pm.v, paw_pm.s,
            paw_rot_pm.v, paw_rot_pm.s, paw_rot_pm.t, paw_rot_pm.ry, paw_rot_pm.rz,
            hip_pm.v, hip_pm.t, hip_pm.r, hip_pm.s,
            pv_pm.v, pv_pm.r, pv_pm.s,
        ]:
            ctl_attr.lock()


    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation between object from guide to rig"""
        self.relatives["root"] = self.ik_hip
        self.controlRelatives["root"] = self.ik_hip

        self.relatives["toe"] = self.toe
        self.controlRelatives["toe"] = self.toe
        self.jointRelatives["toe"] = 4

        # if self.settings["joint"]:
        #     self.jointRelatives["root"] = 0

        self.aliasRelatives["root"] = "ctl"

    def addConnection(self):
        """Add more connection definition to the set"""
        self.connections["standard"] = self.connect_standard
        self.connections["orientation"] = self.connect_orientation

        self.blend_att >> self.flip_node.floatB

        for cns, ik_jnt, fk_jnt in [
            [self.hip_blend_cns, self.hip_ik_joint, self.hip_fk_joint],
            [self.knee_blend_cns, self.knee_ik_joint, self.knee_fk_joint],
            [self.ankle_blend_cns, self.ankle_ik_joint, self.ankle_fk_joint],
            [self.paw_blend_cns, self.paw_ik_joint, self.paw_fk_joint],
            [self.toe_blend_cns, self.toe_ik_joint, self.toe_fk_joint],
            ]:
            pm.connectAttr(self.blend_att, f"{cns}.{ik_jnt.name()}W0")
            pm.connectAttr(self.flip_node.outFloat, f"{cns}.{fk_jnt.name()}W1")


        for ik_ctl in [self.ik_hip, self.ik_paw, self.ik_pv]:
            pm.setAttr(ik_ctl.v, lock=False)
            pm.connectAttr(self.blend_att, ik_ctl.v)

        for fk_ctl in [self.fk_hip, self.fk_knee, self.fk_ankle, self.fk_paw]:
            pm.setAttr(fk_ctl.v, lock=False)
            pm.connectAttr(self.flip_node.outFloat, fk_ctl.v)

    def connect_standard(self):
        """standard connection definition for the component"""
        self.connect_standardWithSimpleIkRef()

    def connect_orientation(self):
        """Orient connection definition for the component"""
        self.connect_orientCns()
