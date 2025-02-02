package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.linearInputToExponential;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "Action Sequance OpMode")
public class ActionSequenceOpMode extends LinearOpMode {

    double joystickTolerance = 0.5;
    Elevators elevators;

    ActionControl control;

    long startTime;

    boolean isSampleMode = true; // does the claw interact with samples (take in / spit) or move around

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        elevators = new Elevators(this);
        elevators.setVerticalPower(0.0);
        control = new ActionControl(elevators, claws, colorSensorSystem, drive, gamepad1, gamepad2);
        claws.setArmTargetPosition(290);
        elevators.setVerticalDestination(0);

        elevators.motorSetHorizontalPower(0.8);

        double epsilon = 0.4;
        double joystickTolerance = 0.05;
        boolean flagDpadDown = true;
        boolean flagElevatorVerticalDpadLeft = true;
        boolean flagDpadUp = true;
        boolean flagDpadRight = true;

        boolean flagX = true;
        boolean flagTriangle = true;
        boolean flagCircle = true;
        boolean flagElevatorHorizontalSquare = true;

        boolean flagClawTakeIn = true;
        boolean ClawState = true;
        boolean clawToggle = true;
        boolean flagClawSpit = false;

        double horElevatorPosition = 0;

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;

        telemetry.addLine("Ready!");
        telemetry.update();
        double lastPower = 0;
        waitForStart();

        startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            /*
            base idea for controls:
                driving is as usual
                systems:
                    claw stuff will be controlled with left_stick_y, toggling using right_trigger
                    picking up is controlled with dpad, and putting in basket stuff with normal buttons
//             */

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -linearInputToExponential(gamepad1.left_stick_y),
                            -linearInputToExponential(gamepad1.left_stick_x)
                    ),
                    -linearInputToExponential(gamepad1.right_stick_x)
            ));

            //if(!control.isOnRun()){
                claws.updateRightClawServoRotation();
                claws.updateLeftClawServoRotation();
            //}

            if (gamepad1.right_trigger >= 0.35) { //split
                claws.rotateWheels(gamepad1.right_trigger);
            }
            else if (gamepad1.left_trigger >= 0.4) {
                claws.rotateWheels(-1);
            }
            else {
                //if(!control.isOnRun()) {
                telemetry.addLine("No Action Run");
                if(System.currentTimeMillis() -startTime < 750)
                    lastPower = claws.getPIDArmPower();

                claws.rotateArm(lastPower);
                //}
//                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.OFF));
//                claws.rotateArm(gamepad1.left_stick_y); //Todo: Maybe remove in the future
            }

            if(gamepad1.square && flagElevatorHorizontalSquare)
                startTime = System.currentTimeMillis();
            flagElevatorHorizontalSquare = !gamepad1.square;


//            if (Math.abs(gamepad1.right_stick_y) > joystickTolerance) {
//                if(horElevatorPosition < 0){
//                    horElevatorPosition = 0;
//                }else if(horElevatorPosition >= Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state){
//                    horElevatorPosition =  Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state;
//                }
//                horElevatorPosition += -gamepad1.right_stick_y*40*3;
//                elevators.motorSetHorizontalDestination((int)(horElevatorPosition));
//            }

            if (gamepad1.dpad_down && flagDpadDown) {
                //control.runAction(control.returnFromDrop());
                Actions.runBlocking(control.returnFromDrop());
            }
            flagDpadDown = !gamepad1.dpad_down;


            if (gamepad1.dpad_up && flagDpadUp) {
                //.runAction(control.getReadyDropHigh());
                Actions.runBlocking(control.getReadyDropHigh());
            }
            flagDpadUp = !gamepad1.dpad_up;

            if (gamepad1.dpad_right && flagDpadRight) {
                //control.runAction(control.getReadyDropLow());
                Actions.runBlocking(control.getReadyDropLow());
            }
            flagDpadRight = !gamepad1.dpad_right;


            if (gamepad1.cross && flagX) {
                //control.runAction(control.returnFromPickUp());
                Actions.runBlocking(control.returnFromPickUp());
            }
            flagX = !gamepad1.cross;

            if (gamepad1.triangle && flagTriangle) {
                //control.runAction(control.getReadyExtendedPickUp());
                Actions.runBlocking(control.getReadyExtendedPickUp());
            }
            flagTriangle = !gamepad1.triangle;

            if (gamepad1.circle && flagCircle) {
                //control.runAction(control.getReadyHalfwayPickUp());
                Actions.runBlocking(control.getReadyHalfwayPickUp());
            }
            flagCircle = !gamepad1.circle;

            if(gamepad1.right_bumper && flagClawSpit){
 //               if(clawToggle)
                    claws.setArmTargetPosition(100);
 //               else
 //                   claws.setArmTargetPosition(0);
                clawToggle = !clawToggle;
                startTime = System.currentTimeMillis();
            }
            flagClawSpit = !gamepad1.right_bumper;
            //double power = pid + ff;
            //if(!control.isOnRun()){
            elevators.updateVert();
            //}
            colorSensorSystem.updateAlert();


            //telemetry.addData("pos: ", claws.getActualArmRotation());
            //telemetry.addData("target: ", claws.getArmTargetPosition());
            telemetry.addData("Right Stick y: ", gamepad1.right_stick_y);
            telemetry.addData("precieved hor position: ", horElevatorPosition);
            telemetry.addData("hor position: ", elevators.motorGetHorizontalPosition());
            telemetry.update();
        }


    }
//    public static void drive(OpMode opMode, MecanumDrive drive){
//        drive.setDrivePowers(new PoseVelocity2d(
//                new Vector2d(
//                        -linearInputToExponential(opMode.gamepad1.left_stick_y),
//                        -linearInputToExponential(opMode.gamepad1.left_stick_x)
//                ),
//                -linearInputToExponential(opMode.gamepad1.right_stick_x)
//        ));
//    }
}
