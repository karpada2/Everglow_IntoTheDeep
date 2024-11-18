package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Servo Test")
public class CRServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo servo = hardwareMap.get(CRServo.class, "claw");

        waitForStart();
//        while (opModeIsActive()) {
//            if (gamepad1.cross) {
//                servo.setPower(0.8);
//            }
//            else if (gamepad1.circle) {
//                servo.setPower(-0.8);
//            }
//            else {
//                servo.setPower(0);
//            }
//        }

    }
}
