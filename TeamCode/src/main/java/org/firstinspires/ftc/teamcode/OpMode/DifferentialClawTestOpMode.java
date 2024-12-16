package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@TeleOp(name="DifferentialClawTestOpMode")
public class DifferentialClawTestOpMode extends LinearOpMode {

    boolean flagCross = false;
    boolean flagCircle = false;
    boolean flagRightBumper = false;
    boolean flagLeftBumper = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);

        waitForStart();

        Action builtAction = null;

        Action sampleAction = null;
        while (opModeIsActive()) {
            if (gamepad2.right_bumper && !flagRightBumper) {
                if (claws.getRotationState() == DifferentialClaws.ClawPowerState.TAKE_IN) {
                    sampleAction = claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF, 100);
                }
                else {
                    sampleAction = claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 100);
                }
            }
            else if (gamepad2.left_bumper && !flagLeftBumper) {
                if (claws.getRotationState() == DifferentialClaws.ClawPowerState.SPIT) {
                    sampleAction = claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF, 100);
                }
                else {
                    sampleAction = claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT, 100);
                }
            }

            if (builtAction == null) {
                builtAction = sampleAction;
            }
            else if (sampleAction != null) {
                builtAction = new SequentialAction(builtAction, sampleAction);
            }

            flagLeftBumper = gamepad2.left_bumper;
            flagRightBumper = gamepad2.right_bumper;


            Action clawMovementAction = null;

            if (gamepad2.cross && !flagCross) {
                clawMovementAction = claws.setClawMovementAction(10);
            }
            else if (gamepad2.circle && !flagCircle) {
                clawMovementAction = claws.setClawMovementAction(0);
            }

            if (builtAction == null) {
                builtAction = clawMovementAction;
            }
            else if (clawMovementAction != null) {
                builtAction = new SequentialAction(builtAction, clawMovementAction);
            }

            if (gamepad2.dpad_down && builtAction != null) {
                Actions.runBlocking(builtAction);
                builtAction = null;
            }
        }
    }
}
