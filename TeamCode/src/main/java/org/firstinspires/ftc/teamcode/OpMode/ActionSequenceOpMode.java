package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.linearInputToExponential;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

public class ActionSequenceOpMode extends LinearOpMode {

    double joystickTolerance = 0.5;
    Elevators elevators;
    DifferentialClaws claws;
    ActionControl control;
    MecanumDrive drive;
    boolean isSampleMode = true; // does the claw interact with samples (take in / spit) or move around

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        elevators = new Elevators(this);
        elevators.setVerticalPower(0.0);
        claws = new DifferentialClaws(this);
        control = new ActionControl(elevators, claws);

        elevators.motorSetHorizontalPower(0.8);

        double epsilon = 0.4;
        double joystickTolerance = 0.05;
        boolean flagDpadDown = true;
        boolean flagElevatorVerticalDpadLeft = true;
        boolean flagDpadUp = true;
        boolean flagElevatorVerticalDpadRight = true;

        boolean flagElevatorHorizontalX = true;
        boolean flagElevatorHorizontalTriangle = true;
        boolean flagElevatorHorizontalCircle = true;
        boolean flagElevatorHorizontalSquare = true;

        boolean flagClawTakeIn = true;
        boolean ClawState = true;
        boolean flagClawSpit = true;

        double horElevatorPosition = 0;

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;

        waitForStart();

        while (opModeIsActive()) {
            /*
            base idea for controls:
                driving is as usual
                systems:
                    claw stuff will be controlled with left_stick_y, toggling using right_trigger
                    picking up is controlled with dpad, and putting in basket stuff with normal buttons
             */
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -linearInputToExponential(gamepad1.left_stick_y),
                            -linearInputToExponential(gamepad1.left_stick_x)
                    ),
                    -linearInputToExponential(gamepad1.right_stick_x)
            ));

            if (gamepad2.right_trigger >= 0.35) { //split
                claws.rotateWheels(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger >= 0.4) {
                claws.rotateWheels(-1);
            }
            else {
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF));
                claws.rotateArm(gamepad2.left_stick_y); //Todo: Maybe remove in the future
            }

            if (Math.abs(gamepad2.right_stick_y) > joystickTolerance) {
                if(horElevatorPosition < 0){
                    horElevatorPosition = 0;
                }else if(horElevatorPosition > 3700){
                    horElevatorPosition =  3700;
                }
                horElevatorPosition += -gamepad2.right_stick_y*40*1.5;
                elevators.motorSetHorizontalDestination((int)(horElevatorPosition));
            }
        }

        if(gamepad2.dpad_down && flagDpadDown) {
            Actions.runBlocking(control.returnFromPickUp);
        }
        flagDpadDown = !gamepad2.dpad_down;

        if(gamepad2.dpad_up && flagDpadUp){
            Actions.runBlocking(control.getReadyDropHigh);
        }
        flagDpadUp = !gamepad2.dpad_up;

        if(gamepad2.dpad_right && flagElevatorVerticalDpadRight){
            Actions.runBlocking(control.getReadyExtendedPickUp);
        }
        flagElevatorVerticalDpadRight = !gamepad2.dpad_right;


        telemetry.addData("Right Stick y: ", gamepad2.right_stick_y);
        telemetry.addData("precieved hor position: ", horElevatorPosition);
        telemetry.addData("hor position: ", elevators.motorGetHorizontalPosition());
        telemetry.update();
    }
}
