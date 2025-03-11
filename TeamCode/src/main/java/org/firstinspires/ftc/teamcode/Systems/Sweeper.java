package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Sweeper {

    Servo sweeper;

    public Sweeper(OpMode opMode) {
        sweeper = opMode.hardwareMap.get(Servo.class, "sweeper");
        sweeper.setDirection(Servo.Direction.FORWARD);
        setPosition(SweeperAngle.SWEEPER_RETRACTED);
    }

    public enum SweeperAngle {
        SWEEPER_RETRACTED(0.42),
        SWEEPER_EXTENDED(0.9);

        public final double angle;

        SweeperAngle(double position) {
            this.angle = position;
        }
    }

    public void setPosition(SweeperAngle position) {
        sweeper.setPosition(position.angle);
    }
    public void setPosition(double position) {sweeper.setPosition(position);}

    // returns the position it is now set to
    public SweeperAngle toggle() {
        if (this.getPosition() == SweeperAngle.SWEEPER_RETRACTED.angle) {
            setPosition(SweeperAngle.SWEEPER_EXTENDED);
            return SweeperAngle.SWEEPER_RETRACTED;
        }
        else {
            setPosition(SweeperAngle.SWEEPER_RETRACTED);
            return SweeperAngle.SWEEPER_EXTENDED;
        }

    }

    public double getPosition() {
        return sweeper.getPosition();
    }
}
