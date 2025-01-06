package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="simple differential test op mode")
public class DifferentialClawsSimpleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CRServo leftClawServo = hardwareMap.get(CRServo.class, "leftClawServo");
        CRServo rightClawServo = hardwareMap.get(CRServo.class, "rightClawServo");
        waitForStart();
        while (opModeIsActive()) {
            leftClawServo.setPower(gamepad1.left_stick_y/2);
            rightClawServo.setPower(gamepad1.right_stick_y/2);
        }
    }
}
