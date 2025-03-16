package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DistanceSensorSystem;

import java.util.WeakHashMap;

@TeleOp(name = "TuningDistanceSensor", group = "Tests")
public class TuningDistanceSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensorSystem distanceSensorSystem = new DistanceSensorSystem(this);
        Pose2d expectedPos = new Pose2d(-24,-11.7, 0);
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("front (INCH)", distanceSensorSystem.getFrontDistance());
            telemetry.addData("right (INCH)", distanceSensorSystem.getRightDistance());
            telemetry.addData("front (INCH)", distanceSensorSystem.isOnSabmersable());
            telemetry.addData("right (INCH)", distanceSensorSystem.isOnRightDistance());
            telemetry.addData("this sould be close as posible to zero", expectedPos.minus(distanceSensorSystem.getPosFromDistanceSensors()));
            telemetry.update();
        }
    }
}
