package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.linearInputToExponential;

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

@TeleOp(name="ActionOpMode")
public class ActionOpMode extends LinearOpMode {

    double triggerThreshold = 0.5;

    Elevators elevators;
    Claws claw;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
//        Actions.runBlocking(
//                new SequentialAction(
//                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_MIN),
//                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_RETRACTED)
//                )
//        );

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = new Claws(this);
        elevators = new Elevators(this);

        boolean flagRightBumper = true;
        boolean flagLeftBumper = true;

        boolean flagTriangle = true;
        boolean flagCircle = true;
        boolean flagSquare = true;
        boolean flagCross = true;

        boolean flagDpadUp = true;
        boolean flagDpadLeft = true;
        boolean flagDpadDown = true;

        Action builtAction = null;

        waitForStart();

        Claws.ClawState currentClawState = Claws.ClawState.OFF;

        while (opModeIsActive()) {
            // driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            linearInputToExponential(-gamepad1.left_stick_y),
                            linearInputToExponential(-gamepad1.left_stick_x)
                    ),
                    linearInputToExponential(-gamepad1.right_stick_x)
            ));

            drive.updatePoseEstimate();

            Action clawAction = null;

            if (gamepad2.right_bumper && flagRightBumper) {
                if (currentClawState == Claws.ClawState.OFF || currentClawState == Claws.ClawState.SPIT) {
                    currentClawState = Claws.ClawState.TAKE_IN;
                    clawAction = claw.setClawAction(Claws.ClawState.TAKE_IN);
                } else {
                    currentClawState = Claws.ClawState.OFF;
                    clawAction = claw.setClawAction(Claws.ClawState.OFF);
                }
            } else if (gamepad2.left_bumper && flagLeftBumper) {
                if (currentClawState == Claws.ClawState.OFF || currentClawState == Claws.ClawState.TAKE_IN) {
                    currentClawState = Claws.ClawState.SPIT;
                    clawAction = claw.setClawAction(Claws.ClawState.SPIT);
                } else {
                    currentClawState = Claws.ClawState.OFF;
                    clawAction = claw.setClawAction(Claws.ClawState.OFF);
                }
            }

            if (builtAction == null) {
                builtAction = clawAction;
            } else {
                builtAction = new SequentialAction(builtAction, clawAction);
            }

            flagRightBumper = !gamepad2.right_bumper;
            flagLeftBumper = !gamepad2.left_bumper;

            Action verticalAction = null;

            if (gamepad2.triangle && flagTriangle) {
                // high basket
                verticalAction = elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH);
            } else if (gamepad2.circle && flagCircle) {
                // low basket
                verticalAction = elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_LOW);
            } else if (gamepad2.cross && flagCross) {
                // hurdle
                verticalAction = elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HURDLE);
            } else if (gamepad2.square && flagSquare) {
                // pickup height
                verticalAction = elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_PICKUP);
            }

            if (builtAction == null) {
                builtAction = verticalAction;
            } else {
                builtAction = new SequentialAction(builtAction, verticalAction);
            }

            flagTriangle = !gamepad2.triangle;
            flagCircle = !gamepad2.circle;
            flagSquare = !gamepad2.square;
            flagCross = !gamepad2.cross;

            Action horizontalAction = null;

            if (gamepad2.dpad_up && flagDpadUp) {
                // most extended
                horizontalAction = elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_EXTENDED.state);
            } else if (gamepad2.dpad_left && flagDpadLeft) {
                // halfway extended
                horizontalAction = elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY.state);
            } else if (gamepad2.dpad_down && flagDpadDown) {
                // most retracted
                horizontalAction = elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED.state);
            }

            if (builtAction == null) {
                builtAction = horizontalAction;
            } else {
                builtAction = new SequentialAction(builtAction, horizontalAction);
            }

            flagDpadLeft = !gamepad2.dpad_left;
            flagDpadUp = !gamepad2.dpad_up;
            flagDpadDown = !gamepad2.dpad_down;

            if (gamepad2.right_trigger < triggerThreshold) {
                if (builtAction != null) {
                    Actions.runBlocking(builtAction);
                    builtAction = null;
                }
            }

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
