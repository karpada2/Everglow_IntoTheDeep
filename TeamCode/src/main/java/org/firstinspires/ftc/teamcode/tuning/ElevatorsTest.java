package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "Elevator Test", group = "Tests")

public class ElevatorsTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = new Elevators(this);
        elevators.setVerticalDestination(0);
        elevators.setVerticalPower(0);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("vert elevators pos:", elevators.getVerticalCurrentPosition());
            telemetry.addData("vert elevators pos:", elevators.getHorizontalPosition());
            telemetry.update();
        }
    }
}
