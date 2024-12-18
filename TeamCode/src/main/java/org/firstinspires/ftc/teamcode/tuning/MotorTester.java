package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "Motor test")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = new Elevators(this);

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad2.right_stick_y / 2; //this cause gamepad sticks give -1 when they are up >:(

            telemetry.addData("current power: ", power);
            telemetry.addData("motor's position: ", elevators.motorGetHorizontalPosition());

            elevators.motorSetHorizontalPower(power);

            telemetry.update();
        }
    }
}
