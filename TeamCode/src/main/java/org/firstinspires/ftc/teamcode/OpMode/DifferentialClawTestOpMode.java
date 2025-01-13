package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@TeleOp(name="DifferentialClawTestOpMode")
@Disabled
public class DifferentialClawTestOpMode extends LinearOpMode {

    boolean flagCircle = false;
    boolean flagSquare = false;
    boolean flagTriangle = false;

    boolean flagDpadUp = false;
    boolean flagDpadDown = false;


    double expected_arm_position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);

        waitForStart();

        Action builtAction = null;

        Action sampleAction = null;
        while (opModeIsActive()) {
            if (gamepad2.circle && !flagCircle) {
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN));
            }
            else if (gamepad2.square && !flagSquare) {
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF));
            }
            else if (gamepad2.triangle && !flagTriangle) {
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT));
            }
            else if (gamepad2.dpad_up && !flagDpadUp) {
                Actions.runBlocking(claws.addClawMovementAction(10));
                expected_arm_position += 10;
            }
            else if (gamepad2.dpad_down && !flagDpadDown) {
                Actions.runBlocking(claws.addClawMovementAction(-10));
                expected_arm_position += -10;
            }
            flagSquare = gamepad2.square;
            flagCircle = gamepad2.circle;
            flagTriangle = gamepad2.triangle;

            flagDpadUp = gamepad2.dpad_up;
            flagDpadDown = gamepad2.dpad_down;

            telemetry.addData("left servo rotation: ", claws.getleftClawServoRotation());
            telemetry.addData("right servo rotation: ", claws.getrightClawServoRotation());
            telemetry.addData("artificial arm position: ", claws.getClawRotation());
            telemetry.addData("expected arm position: ", expected_arm_position);
            telemetry.update();
        }
    }
}
