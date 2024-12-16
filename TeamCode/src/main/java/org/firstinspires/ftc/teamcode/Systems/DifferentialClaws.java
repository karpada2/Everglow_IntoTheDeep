/*
--------------------------------------------------------------------------
| This is going to work by internally keeping each servo's rotation in   |
| degrees (0 is starting) and possibly the entire claw's position.       |
| Actions must be used so that the servo actually reaches its position.  |
--------------------------------------------------------------------------
*/

package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DifferentialClaws {

    public static double getRotationOfInput(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360;
    }

    CRServo servo1;
    CRServo servo2;
    AnalogInput clawInput1;
    AnalogInput clawInput2;

    // tracks from -∞ - ∞ the rotation of each motor.
    double servo1Rotation = 0;
    double servo2Rotation = 0;

    public class ClawMovementAction implements Action {
        private final double destination; // in degrees, where 0 is the starting degrees

        private final int direction;
        private boolean isInit = false;

        private final double power = 0.5;
        private final double tolerance = 0.1; //in degrees, how much error can be accepted

        public ClawMovementAction(double destination) {
            this.destination = destination;
            if (getClawRotation() < destination) {
                direction = 1;
            }
            else {
                direction = -1;
            }
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInit) {
                rotateArm(power * direction);
                isInit = true;
            }

            if (Math.abs(getClawRotation() - destination) < tolerance) {
                rotateArm(0);
                return false;
            }
            return true;
        }
    }

    // receives the time in milliseconds until the action is considered finished
    public class ClawSampleInteractionAction implements Action {
        private final double wantedPower;
        private final double timeUntilFinished;
        private double startTime;
        private boolean isInitialized = false;

        public ClawSampleInteractionAction(ClawPowerState state, double timeToStop) {
            this.wantedPower = state.state;
            this.timeUntilFinished = timeToStop;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInitialized) {
                rotateWheels(wantedPower);
                startTime = System.currentTimeMillis();
                isInitialized = true;
            }

            return System.currentTimeMillis() - startTime < timeUntilFinished;
        }
    }

    public DifferentialClaws(OpMode opMode) {
        servo1 = opMode.hardwareMap.get(CRServo.class, "clawServo1");
        servo2 = opMode.hardwareMap.get(CRServo.class, "clawServo2");
        clawInput1 = opMode.hardwareMap.get(AnalogInput.class, "clawAnalogInput1");
        clawInput2 = opMode.hardwareMap.get(AnalogInput.class, "clawAnalogInput2");

        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    enum ClawPowerState {
        TAKE_IN(0.5),
        OFF(0),
        SPIT(-0.5);

        public final double state;

        ClawPowerState(double state) {this.state = state;}
    }

    public void rotateArm(double power){
        power /= 2;
        servo1.setPower(power);
        servo2.setPower(-power);
    }

    public void rotateWheels(double power) {
        servo1.setPower(power);
        servo2.setPower(power);
    }

    public void setPower(double p1, double p2){
        double sum = p1+p2;
        p1 /= sum;
        p2 /= sum;

        servo1.setPower(p1);
        servo2.setPower(p2);
    }

    public double getServo1Rotation() {
        return getRotationOfInput(clawInput1);
    }

    public double getServo2Rotation() {
        return getRotationOfInput(clawInput2);
    }

    public double getClawRotation() {
        return (getServo1Rotation() + getServo2Rotation())/2.0;
    }


    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state, double timeUntilFinished) {
        return new ClawSampleInteractionAction(state, timeUntilFinished);
    }

    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state) {
        return new ClawSampleInteractionAction(state, 0);
    }
}
