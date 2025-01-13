package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "Elevator Test")
public class ElevatorsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = new Elevators(this);

        waitForStart();
        double pos = 0;
        while (opModeIsActive()){
            pos += -gamepad1.left_stick_y*10;

            elevators.setVertDest((int)pos);
        }
    }
}
