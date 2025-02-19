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
        boolean flagElevatorVerticalDpadUp = true;
        boolean flagElevatorVerticalDpadDown = true;

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("vert elevators pos:", elevators.getVerticalCurrentPosition());
            telemetry.addData("hor elevators pos:", elevators.getHorizontalPosition());
            telemetry.addData("vert left vel:", elevators.getLeftVelocity());
            telemetry.addData("vert right vel:", elevators.getRightVelocity());
            telemetry.update();
            if (gamepad2.square){
                elevators.setVerticalDestination(0);
                elevators.setVerticalPower(0);
            }

            if(gamepad2.dpad_up && flagElevatorVerticalDpadUp){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_LOW.state);
            }
            flagElevatorVerticalDpadUp = !gamepad2.dpad_up;

            if(gamepad2.dpad_down && flagElevatorVerticalDpadDown) {
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_PICKUP.state);
            }
            flagElevatorVerticalDpadDown = !gamepad2.dpad_down;
        }
    }
}
