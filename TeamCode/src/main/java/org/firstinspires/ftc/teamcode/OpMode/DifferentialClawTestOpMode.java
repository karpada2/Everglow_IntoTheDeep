package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@TeleOp(name="DifferentialClawTestOpMode")
public class DifferentialClawTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);

        waitForStart();
        double powerLeft;
        double powerRight;
        while (opModeIsActive()) {
            powerLeft = -gamepad2.left_stick_y/5;
            powerRight = -gamepad2.right_stick_y/5;
            //claws.rotateArm(power);
            //claws.rotateWheels();
        }
    }
}
