package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="base Differential test op mode")
public class DifferentialClawBaseTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.get(CRServo.class, "CRServo");
        AnalogInput inpt = hardwareMap.get(AnalogInput.class, "AxonAnalog1");
        boolean flagTriangle = true;
        double startPos = getDegrees(inpt);
        double targetPos = startPos;
        boolean moveToPosition = false;
        double tolerance = 2.5;
        servo.setPower(0);
        servo.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        double startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            startPos = getDegrees(inpt);

            if (gamepad1.square) {
                servo.setPower(0.5);
            }
            if (gamepad1.circle) {
                servo.setPower(0);
            }


            if (gamepad1.triangle && flagTriangle) {
                targetPos = (startPos + 30) % 360;
                moveToPosition = true;
                servo.setPower(0.4);
            }
            flagTriangle = !gamepad1.triangle;

            if (Math.abs(getDegrees(inpt) - targetPos) <= tolerance && moveToPosition) {
                moveToPosition = false;
                servo.setPower(0);
            }

            telemetry.addData("targetPos: ", targetPos);
            telemetry.addData("curr pos: ", getDegrees(inpt));
            telemetry.addData("curr power: ", servo.getPower());
            telemetry.update();
        }
    }

    public static double getDegrees(AnalogInput input) {
        return (input.getVoltage()/ input.getMaxVoltage()) * 360;
    }
}