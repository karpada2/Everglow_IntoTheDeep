package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.linearInputToExponential;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Disabled
@TeleOp(name="ActionOpMode")
public class ActionOpMode extends LinearOpMode {

    double triggerThreshold = 0.5;

    Elevators elevators;
    DifferentialClaws claw;
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
        claw = new DifferentialClaws(this);
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
        DifferentialClaws.ClawPowerState currentClawPowerState = DifferentialClaws.ClawPowerState.OFF;

        while (opModeIsActive()) {
            // driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            linearInputToExponential(gamepad1.left_stick_y),
                            linearInputToExponential(gamepad1.left_stick_x)
                    ),
                    linearInputToExponential(gamepad1.right_stick_x)
            ));

            drive.updatePoseEstimate();

            Action clawAction = null;

            if (gamepad2.right_bumper && flagRightBumper) {
                if (currentClawPowerState == DifferentialClaws.ClawPowerState.OFF || currentClawPowerState == DifferentialClaws.ClawPowerState.SPIT) {
                    currentClawPowerState = DifferentialClaws.ClawPowerState.TAKE_IN;
                    clawAction = claw.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN);
                } else {
                    currentClawPowerState = DifferentialClaws.ClawPowerState.OFF;
                    clawAction = claw.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF);
                }
            } else if (gamepad2.left_bumper && flagLeftBumper) {
                if (currentClawPowerState == DifferentialClaws.ClawPowerState.OFF || currentClawPowerState == DifferentialClaws.ClawPowerState.TAKE_IN) {
                    currentClawPowerState = DifferentialClaws.ClawPowerState.SPIT;
                    clawAction = claw.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT);
                } else {
                    currentClawPowerState = DifferentialClaws.ClawPowerState.OFF;
                    clawAction = claw.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF);
                }
            }

            if (builtAction == null) {
                builtAction = clawAction;
            } else if (clawAction != null) {
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
            } else if (verticalAction != null) {
                builtAction = new SequentialAction(builtAction, verticalAction);
            }

            flagTriangle = !gamepad2.triangle;
            flagCircle = !gamepad2.circle;
            flagSquare = !gamepad2.square;
            flagCross = !gamepad2.cross;

            Action horizontalAction = null;

            if (gamepad2.dpad_up && flagDpadUp) {
                // most extended
                horizontalAction = elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED);
            } else if (gamepad2.dpad_left && flagDpadLeft) {
                // halfway extended
                horizontalAction = elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY);
            } else if (gamepad2.dpad_down && flagDpadDown) {
                // most retracted
                horizontalAction = elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED);
            }

            if (builtAction == null) {
                builtAction = horizontalAction;
            } else if (horizontalAction != null) {
                builtAction = new SequentialAction(builtAction, horizontalAction);
            }

            flagDpadLeft = !gamepad2.dpad_left;
            flagDpadUp = !gamepad2.dpad_up;
            flagDpadDown = !gamepad2.dpad_down;

            if (gamepad2.right_trigger > triggerThreshold) {
                if (builtAction != null) {
                    Actions.runBlocking(builtAction);
                    builtAction = null;
                }
            }

            telemetry.update();
        }
    }
}
