package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;

@TeleOp(name = "TestFloorSensors", group = "Tests")
@Config
public class TestFloorSensors extends LinearOpMode {
    public static double power = 0.3;
    public static double breakPower = 0;
    public static double afterPower = -0.3;

    public static double redConst = 0;

    public static double blueConst = 0;

    public static double eps = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        waitForStart();


        boolean recLeft;
        boolean recRight;
        boolean lastRec1=false, lastRec2=false;
        int timesOnTheLine1 = 0, timesOnTheLine2 = 0;
        double redRec, blueRec;

//        drive.frontLeft.setPower(power);
//        drive.backLeft.setPower(power);
//        drive.frontRight.setPower(power);
//        drive.backRight.setPower(power);

        while (opModeIsActive()){
            recLeft = colorSensorSystem.isOnTape(true);
            recRight = colorSensorSystem.isOnTape(false);

            timesOnTheLine1 += recRight != lastRec1? 1: 0;
            timesOnTheLine2 += recLeft != lastRec2? 1: 0;

            redRec = colorSensorSystem.leftSensor.red();
            blueRec = colorSensorSystem.leftSensor.blue();

            //TODO: run tuning, change
            telemetry.addData("rightSensor", recRight);
            telemetry.addData("leftSensor", recLeft);
            telemetry.addData("red:", redRec);
            //telemetry.addData("red:", colorSensorSystem.rightSensor.red());
            telemetry.addData("blue", blueRec);
            telemetry.addData("alpha", colorSensorSystem.leftSensor.alpha());
            telemetry.addData("green", colorSensorSystem.leftSensor.green());
            telemetry.addData("is on red?", Math.abs(redRec - redConst) <= eps);
            telemetry.addData("is on blue?", Math.abs(blueRec - blueConst) <= eps);

            //TODO: find way (with the parameters above) to make difference between recognition of blue and red to white
            //TODO: put your code here:
            telemetry.addData("is on white? (should be false when on red or blue)", "Put here code");

            telemetry.update();

            //TODO: finished tuning? now update the ColorSensorSystem and run the blow code to check if it works
//
//            switch (timesOnTheLine2 % 3){
//                case 0:
//                    drive.frontLeft.setPower(power);
//                    drive.backLeft.setPower(power);
//                    break;
//                case 1:
//                    drive.frontLeft.setPower(0);
//                    drive.backLeft.setPower(0);
//                    break;
//                default:
//                    drive.frontLeft.setPower(afterPower);
//                    drive.backLeft.setPower(afterPower);
//                    break;
//            }
//
//            switch (timesOnTheLine1 % 3){
//                case 0:
//                    drive.frontRight.setPower(power);
//                    drive.backRight.setPower(power);
//                    break;
//                case 1:
//                    drive.frontRight.setPower(0);
//                    drive.backRight.setPower(0);
//                    break;
//                default:
//                    drive.frontRight.setPower(afterPower);
//                    drive.backRight.setPower(afterPower);
//                    break;
//            }

            lastRec2 = recLeft;
            lastRec1 = recRight;
        }
    }
}
;