package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="Sample BLUE opMode", group = "AAASampleOpModes")
public class SampleBlueOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleOpMode opMode = new SampleOpMode(this, gamepad1, gamepad2);
        opMode.run(true);
    }
}
