package org.firstinspires.ftc.teamcode.tuning;

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
public class TestFloorSensors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));

        waitForStart();

        double frontLeftPower = 0.5
                , frontRightPower = 0.5
                , rearLeftPower = 0.5
                ,rearRightPower = 0.5;
        boolean isOne, isTwo;

        while (opModeIsActive()){
            isOne = colorSensorSystem.isOnTape(true);
            isTwo = colorSensorSystem.isOnTape(false);
            telemetry.addData("rightSensor", isOne);
            telemetry.addData("leftSensor", isTwo);
            telemetry.addData("red:", colorSensorSystem.leftSensor.red());
            telemetry.addData("red:", colorSensorSystem.rightSensor.red());
//            telemetry.addData("blue", colorSensorSystem.leftSensor.blue());
//            telemetry.addData("alpha:", colorSensorSystem.leftSensor.alpha());
//            telemetry.addData("green", colorSensorSystem.leftSensor.green());
            telemetry.update();

            drive.backLeft.setPower(rearLeftPower);
            drive.backRight.setPower(rearRightPower);
            drive.frontLeft.setPower(frontLeftPower);
            drive.frontRight.setPower(frontRightPower);

            if (isOne){
                frontRightPower = 0;
                rearRightPower = 0;
            }
            if (isTwo){
                frontLeftPower = 0;
                rearLeftPower = 0;
            }

        }
    }
}
;