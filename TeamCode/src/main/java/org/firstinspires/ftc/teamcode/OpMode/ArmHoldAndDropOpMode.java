package org.firstinspires.ftc.teamcode.OpMode;


import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@TeleOp(name="arm hold and drop test op mode")
public class ArmHoldAndDropOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);

        waitForStart();

        Actions.runBlocking(claws.test(2000, 1000));

        while (opModeIsActive()){

        }
    }
}
