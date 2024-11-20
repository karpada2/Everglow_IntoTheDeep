package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Claws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;


@TeleOp(name="Action OpMode")
public class ActionNewOpMode extends LinearOpMode {

    double trigger_threshold = 0.5;

    Elevators elevators = new Elevators(this);
    Claws claw = new Claws(this);
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    @Override
    public void runOpMode() throws InterruptedException {
        Actions.runBlocking(
                new SequentialAction(
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_MIN),
                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_RETRACTED)
                )
        );

        boolean flagRightBumper = true;
        boolean flagLeftBumper = true;

        boolean flagTriangle = true;
        boolean flagCircle = true;

        boolean flagDpadUp = true;
        boolean flagDpadLeft = true;

        waitForStart();

        Claws.ClawState currentClawState = Claws.ClawState.OFF;

        while (opModeIsActive()) {
            // driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y/2,
                            -gamepad1.left_stick_x/2
                    ),
                    -gamepad1.right_stick_x/2
            ));

            drive.updatePoseEstimate();



            if (gamepad2.right_bumper && flagRightBumper) {
                if (currentClawState == Claws.ClawState.OFF || currentClawState == Claws.ClawState.SPIT) {
                    currentClawState = Claws.ClawState.TAKE_IN;
                }
                else {
                    currentClawState = Claws.ClawState.OFF;
                }
            }
            else if (gamepad2.left_bumper && flagLeftBumper) {
                if (currentClawState == Claws.ClawState.OFF || currentClawState == Claws.ClawState.TAKE_IN) {
                    currentClawState = Claws.ClawState.SPIT;
                }
                else {
                    currentClawState = Claws.ClawState.OFF;
                }
            }

            flagRightBumper = !gamepad2.right_bumper;
            flagLeftBumper = !gamepad2.left_bumper;

            Action clawAction = claw.setClawAction(currentClawState);

            Action hurdleAction = null;

            if (gamepad2.triangle && flagTriangle) {
                hurdleAction = new SequentialAction(
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_HURDLE),
                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_HALFWAY),
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_PICKUP)
                );
            }
            else if (gamepad2.circle && flagCircle) {
                hurdleAction = new SequentialAction(
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_HURDLE),
                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_PICKUP)
                );
            }

            flagTriangle = !gamepad2.triangle;
            flagCircle = !gamepad2.circle;

            Action basketAction = null;

            if (gamepad2.dpad_left && flagDpadLeft) {
                basketAction = new SequentialAction(
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_LOW),
                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_HALFWAY),
                        claw.clawSpit()
                );
            }
            else if (gamepad2.dpad_up && flagDpadUp) {
                basketAction = new SequentialAction(
                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_HIGH),
                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_HALFWAY),
                        claw.clawSpit()
                );
            }

            flagDpadLeft = !gamepad2.dpad_left;
            flagDpadUp = !gamepad2.dpad_up;

            /*
            right bumper - claw takeIn
            left bumper - claw spit
            triangle - action to get ready for pickup (clear hurdle and that)
            circle - action to clear hurdle in the opposite way
            dpad up - action to move to low basket and put the sample in
            dpad left - action to move to high basket and put the sample in
             */
        }
    }
}

