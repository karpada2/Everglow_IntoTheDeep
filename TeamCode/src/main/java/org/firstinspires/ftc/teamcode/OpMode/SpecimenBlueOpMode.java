package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Specimen BLUE opMode", group = "AAASpecimenOpModes")
public class SpecimenBlueOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SpecimenOpMode opMode = new SpecimenOpMode(this, gamepad1, gamepad2);
        opMode.run(true);
    }
}
