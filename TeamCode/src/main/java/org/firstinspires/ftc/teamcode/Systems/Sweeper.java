package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Sweeper {
    public double angleToServoPosition(double angle) {
        return angle/max_movement;
    }

    public double servoPositionToAngle(double position) {
        return position*max_movement;
    }

    Servo sweeper;

    public final double max_movement = 300.0;

    public Sweeper(OpMode opMode) {
        sweeper = opMode.hardwareMap.get(Servo.class, "sweeper");
    }

    public enum SweeperAngle {
        SWEEPER_RETRACTED(0.0),
        SWEEPER_EXTENDED(120.0);

        public final double angle;

        SweeperAngle(double position) {
            this.angle = position;
        }
    }

    public void setAngle(SweeperAngle angle) {
        setAngle(angle.angle);
    }

    public void setAngle(double angle) {
        setPosition(angleToServoPosition(angle));
    }

    public void setPosition(double position) {
        sweeper.setPosition(position);
    }


    public double getPosition() {
        return sweeper.getPosition();
    }

    public double getAngle() {
        return servoPositionToAngle(getPosition());
    }
}
