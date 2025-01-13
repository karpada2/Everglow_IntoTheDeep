package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
@TeleOp(name="base Differential test op mode")
@Disabled
public class DifferentialClawBaseTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftClawServo = hardwareMap.get(CRServo.class, "leftClawServo");
        CRServo rightClawServo = hardwareMap.get(CRServo.class, "rightClawServo");
        waitForStart();
        while (opModeIsActive()) {
            if (Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.right_stick_y)) {
                leftClawServo.setPower(gamepad1.left_stick_y / 2);
                rightClawServo.setPower(gamepad1.left_stick_y / 2);
            }
            else {
                leftClawServo.setPower(gamepad1.right_stick_y / 2);
                rightClawServo.setPower(-gamepad1.right_stick_y / 2);
            }
        }
    }
}