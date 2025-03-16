package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "Motor test", group="Tests")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        GamepadEx gamepad = new GamepadEx(gamepad1);

        waitForStart();

        int timesFrontLeftPressed = 0;
        int timesFrontRightPressed = 0;
        int timesBackLeftPressed = 0;
        int timesBackRightPressed = 0;

        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = 0;


        while (opModeIsActive()) {
            gamepad.readButtons();
            if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                timesBackLeftPressed++;
            }
            else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                timesFrontLeftPressed++;
            }
            else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                timesFrontRightPressed++;
            }
            else if (gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                timesBackRightPressed++;
            }

            timesFrontLeftPressed = timesFrontLeftPressed % 3;
            timesFrontRightPressed = timesFrontRightPressed % 3;
            timesBackLeftPressed = timesBackLeftPressed % 3;
            timesBackRightPressed = timesBackRightPressed % 3;

            if (timesFrontLeftPressed == 0) {
                frontLeftPower = 0;
            }
            else if (timesFrontLeftPressed == 1) {
                frontLeftPower = 0.5;
            }
            else {
                frontLeftPower = -0.5;
            }

            if (timesFrontRightPressed == 0) {
                frontRightPower = 0;
            }
            else if (timesFrontRightPressed == 1) {
                frontRightPower = 0.5;
            }
            else {
                frontRightPower = -0.5;
            }

            if (timesBackLeftPressed == 0) {
                backLeftPower = 0;
            }
            else if (timesBackLeftPressed == 1) {
                backLeftPower = 0.5;
            }
            else {
                backLeftPower = -0.5;
            }

            if (timesBackRightPressed == 0) {
                backRightPower = 0;
            }
            else if (timesBackRightPressed == 1) {
                backRightPower = 0.5;
            }
            else {
                backRightPower = -0.5;
            }

            if (gamepad.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                drive.backLeft.setPower(backLeftPower);
                drive.frontLeft.setPower(frontLeftPower);
                drive.frontRight.setPower(frontRightPower);
                drive.backRight.setPower(backRightPower);
            }
            else {
                drive.backLeft.setPower(0);
                drive.frontLeft.setPower(0);
                drive.frontRight.setPower(0);
                drive.backRight.setPower(0);
            }

            telemetry.addData("backLeft", backLeftPower);
            telemetry.addData("frontLeft", frontLeftPower);
            telemetry.addData("frontRight", frontRightPower);
            telemetry.addData("backRight", backRightPower);
            telemetry.addData("backLeftPresses", timesBackLeftPressed);
            telemetry.addData("frontLeftPresses", timesFrontLeftPressed);
            telemetry.addData("frontRightPresses", timesFrontRightPressed);
            telemetry.addData("backRightPresses", timesBackRightPressed);
            telemetry.update();
        }
    }
}
