package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSystem {
    DistanceSensor frontDistance;
    DistanceSensor sideDistance;

    //TODO: Check what is the min distance and update that
    public static final double MIN_DISTANCE = 0;

    private double expectedDistance = 5.5;
    public DistanceSensorSystem(OpMode opMode)
    {
        frontDistance = opMode.hardwareMap.get(DistanceSensor.class, "frontDistance");
        sideDistance = opMode.hardwareMap.get(DistanceSensor.class, "sideDistance");
    }

    public boolean isOnSabmersable(){
        return getFrontDistance() <= MIN_DISTANCE;
    }

    public boolean isOnRightDistance(){
        return getRightDistance() <= expectedDistance;
    }

    public double getRightDistance(){
        return sideDistance.getDistance(DistanceUnit.INCH);
    }

    public double getFrontDistance(){
        return frontDistance.getDistance(DistanceUnit.INCH);
    }
}
