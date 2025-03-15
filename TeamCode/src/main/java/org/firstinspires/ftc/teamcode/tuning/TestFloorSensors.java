package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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
    public static double afterPower = 0.1;

    public static double redConst = 1420;

    public static double blueConst = 2960;

    public static double whiteGreenValueConst = 6600;

    public static double eps = 700;

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, false);
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(10,10,0));

        waitForStart();

        double angle = 0;


        boolean recLeft;
        boolean recRight;

        boolean leftDetected = false;
        boolean rightDetected = false;

        int turnMultiplier = 0;

        boolean finishedCalculatingRotation = false;
        boolean startedRotating = false;

        while (opModeIsActive()){
            recLeft = colorSensorSystem.isOnTape(false);
            recRight = colorSensorSystem.isOnTape(true);

            if (recLeft && !leftDetected) {
                leftDetected = true;
                if (!rightDetected) {
                    angle = drive.pose.heading.toDouble();
                }
            }
            if (recRight && !rightDetected) {
                rightDetected = true;
                if (!leftDetected) {
                    angle = drive.pose.heading.toDouble();
                }
            }


//            //TODO: run tuning, change
//            telemetry.addData("rightSensor", recRight);
//            telemetry.addData("leftSensor", recLeft);
//            telemetry.addData("red:", redRec);
//            //telemetry.addData("red:", colorSensorSystem.rightSensor.red());
//            telemetry.addData("blue", blueRec);
//            telemetry.addData("alpha", colorSensorSystem.leftSensor.alpha());
//            telemetry.addData("green", colorSensorSystem.leftSensor.green());
//            telemetry.addData("is on red?", Math.abs(redRec - redConst) <= eps);
//            telemetry.addData("is on blue?", Math.abs(blueRec - blueConst) <= eps);
//
//            //TODO: find way (with the parameters above) to make difference between recognition of blue and red to white
//            //TODO: put your code here:
//            telemetry.addData("is on white? (should be false when on red or blue)", Math.abs(colorSensorSystem.leftSensor.green() - whiteGreenValueConst) <= eps);
//
//            telemetry.update();
//
//            telemetry.addData("time left left the line", timeLeftTheLineLeft);
//            telemetry.addData("time right left the line", timeLeftTheLineRight);
//
//            telemetry.update();



            //TODO: finished tuning? now update the ColorSensorSystem and run the blow code to check if it works

            if (finishedCalculatingRotation) {
                if (!startedRotating) {
                    startedRotating = true;
                    if (!(recLeft && recRight)) {
                        Actions.runBlocking(drive.actionBuilder(drive.pose).fresh().turn(turnMultiplier * angle).build());
                    }
                }
            }
            else {
                if (!leftDetected && !rightDetected) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    power,
                                    0
                            ),
                            0
                    ));
                }
                else if (rightDetected) {
                    if (leftDetected) {
                        turnMultiplier = -1;
                        angle = drive.pose.heading.toDouble() - angle;
                        angle /= 2.0;
                        finishedCalculatingRotation = true;
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0
                        ));
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                -0.2
                        ));
                    }
                }
                else if (leftDetected) {
                    if (rightDetected) {
                        turnMultiplier = 1;
                        angle = drive.pose.heading.toDouble() - angle;
                        angle /= 2.0;
                        finishedCalculatingRotation = true;
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0
                        ));
                    } else {
                        drive.setDrivePowers(new PoseVelocity2d(
                                new Vector2d(
                                        0,
                                        0
                                ),
                                0.2
                        ));
                    }
                }
                drive.updatePoseEstimate();
            }

            telemetry.addData("started rotating", startedRotating);
            telemetry.addData("left detected", leftDetected);
            telemetry.addData("right detected", rightDetected);
            telemetry.addData("angle to turn", angle);
            telemetry.addData("robot angle", drive.pose.heading.toDouble());
            telemetry.update();


            // blue tape: red: 364, blue: 2960, green: 1111, alpha: 1486
            // red tape: red: 1417, blue: 709, green: 984, alpha: 1044
            // white tape: red: 3222, blue: 7126, green: 6650, alpha: 5710
        }
    }
}
;