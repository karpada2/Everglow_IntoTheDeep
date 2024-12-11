package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="DifferentialClawTestOpMode")
public class DifferentialClawTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo1 = hardwareMap.get(CRServo.class, "testClawServo1");
        CRServo servo2 = hardwareMap.get(CRServo.class, "testClawServo2");

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPower(gamepad1.left_stick_y/2);
            servo2.setPower(gamepad1.right_stick_y/2);
        }
    }
}
