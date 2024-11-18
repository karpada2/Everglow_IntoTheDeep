package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

        waitForStart();

        Action controllerAction = null; // person controlling the elevators and claws

        while (opModeIsActive()) {
            // driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            Action lastAction = null;

            if (gamepad2.dpad_down) {
                controllerAction = elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_PICKUP);
            }
            else if (gamepad2.dpad_left) {
                controllerAction = elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_HURDLE);
            }
            else if (gamepad2.dpad_up) {
                controllerAction = elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_LOW);
            }
            else if (gamepad2.dpad_right) {
                controllerAction = elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_HIGH);
            }

            if (gamepad2.cross) {
                lastAction = elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_EXTENDED);
            }
            else if (gamepad2.triangle) {
                lastAction = elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_RETRACTED);
            }
            else if (gamepad2.square) {
                lastAction = elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_DROP);
            }
            else if (gamepad2.circle) {
                lastAction = elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_HALFWAY);
            }

            if (lastAction != null) {
                if (controllerAction != null) {
                    controllerAction = new ParallelAction(controllerAction, lastAction);
                }
                else {
                    controllerAction = lastAction;
                }
            }

            if (gamepad2.right_trigger > trigger_threshold) {
                lastAction = claw.takeIn();
            }
            else if (gamepad2.right_bumper) {
                lastAction = claw.clawSpit();
            }
            else {
                lastAction = claw.turnOff();
            }

            if (controllerAction != null) {
                controllerAction = new ParallelAction(controllerAction, lastAction);
            }
            else {
                controllerAction = lastAction;
            }

            Actions.runBlocking(controllerAction);
        }
    }
}

