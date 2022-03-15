import mgear.shifter.custom_step as cstp
import maya.cmds as cmds


class CustomShifterStep(cstp.customShifterMainStep):
    """Custom Step description
    """

    def setup(self):
        """
        Setting the name property makes the custom step accessible
        in later steps.

        i.e: Running  self.custom_step("HideGuide")  from steps ran after
             this one, will grant this step.
        """
        self.name = "HideGuide"

    def run(self):
        """Run method.

            i.e:  self.mgear_run.global_ctl
                gets the global_ctl from shifter rig build base

            i.e:  self.component("control_C0").ctl
                gets the ctl from shifter component called control_C0

            i.e:  self.custom_step("otherCustomStepName").ctlMesh
                gets the ctlMesh from a previous custom step called
                "otherCustomStepName"

        Returns:
            None: None
        """
        cmds.hide("guide")
        return
