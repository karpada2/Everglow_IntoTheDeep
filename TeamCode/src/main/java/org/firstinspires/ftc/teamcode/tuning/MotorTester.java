package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Elevators;


@TeleOp(name = "Motor test", group="Tests")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = new Elevators(this);
        double horPosition = elevators.motorGetHorizontalPosition();
        double vertPosition = elevators.getVerticalCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {
            //horPosition += -gamepad2.left_stick_x*10;
            vertPosition += -gamepad2.left_stick_y*10;

            telemetry.addData("horizontal elevator's position", elevators.motorGetHorizontalPosition());
            telemetry.addData("vertical elevator's position", elevators.getVerticalCurrentPosition());
            telemetry.addData("horizontal elevator's wanted pos", horPosition);
            telemetry.addData("vertical elevator's wanted pos", vertPosition);

            elevators.motorSetHorizontalDestination((int)horPosition);
            elevators.setVerticalDestination((int)vertPosition);

            telemetry.update();
        }
    }
}
