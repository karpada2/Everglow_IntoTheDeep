package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sample RED opMode", group = "AAASampleOpModes")
public class SampleRedOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleOpMode opMode = new SampleOpMode(this, gamepad1, gamepad2);
        opMode.run(false);
    }
}
