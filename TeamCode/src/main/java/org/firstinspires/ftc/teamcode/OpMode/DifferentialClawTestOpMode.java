package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@TeleOp(name="DifferentialClawTestOpMode")
public class DifferentialClawTestOpMode extends LinearOpMode {

    boolean flagCross = false;
    boolean flagCircle = false;
    boolean flagSquare = false;
    boolean flagTriangle = false;
    boolean flagRightBumper = false;
    boolean flagLeftBumper = false;

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
            else if (gamepad2.cross && !flagCross) {
                Actions.runBlocking(claws.setClawMovementAction(10));
            }
            flagCross = gamepad2.cross;
            flagSquare = gamepad2.square;
            flagCircle = gamepad2.circle;
            flagTriangle = gamepad2.triangle;

            telemetry.addData("left servo rotation: ", claws.getleftClawServoRotation());
            telemetry.addData("right servo rotation: ", claws.getrightClawServoRotation());
            telemetry.update();
        }
    }
}
