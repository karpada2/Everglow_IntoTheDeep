package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DistanceSensorSystem;

import java.util.WeakHashMap;

@TeleOp(name = "TuningDistanceSensor", group = "Tests")
public class TuningDistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensorSystem distanceSensorSystem = new DistanceSensorSystem(this);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("front (INCH)", distanceSensorSystem.getFrontDistance());
            telemetry.addData("right (INCH)", distanceSensorSystem.getRightDistance());
            telemetry.addData("front (INCH)", distanceSensorSystem.isOnSabmersable());
            telemetry.addData("right (INCH)", distanceSensorSystem.isOnRightDistance());
            telemetry.update();
        }
    }
}
