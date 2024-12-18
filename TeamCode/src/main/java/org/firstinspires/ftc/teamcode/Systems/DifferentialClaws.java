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

    double armPosition = 0;

    // tracks from -∞ - ∞ the rotation of each motor.
    double servo1Rotation = 0;
    double servo2Rotation = 0;

    private ClawPowerState wheelRotationState;

    public class ClawMovementAction implements Action {
        private double servo1Destination;// in degrees, where 0 is the starting degrees
        private double servo2Destination;

        private final int directionServo1;
        private final int directionServo2;
        private boolean isInitialized = false;

        private final double power = 0.5;
        private final double tolerance = 0.1; //in degrees, how much error can be accepted

        public ClawMovementAction(double destination) {
            this.servo1Destination = (getServo1Rotation() + destination);
            this.servo2Destination = (getServo2Rotation() + destination);

            directionServo1 = getServo1Rotation() < servo1Destination ? 1 : -1;
            directionServo2 = getServo2Rotation() < servo2Destination ? 1 : -1;

            servo1Destination = servo1Destination % 360;
            servo2Destination = servo2Destination % 360;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInitialized) {
                servo1.setPower(power * directionServo1);
                servo2.setPower(-power * directionServo2);
                isInitialized = true;
            }

            if (Math.abs(getServo1Rotation() - servo1Destination) < tolerance || Math.abs(getServo2Rotation() - servo2Destination) < tolerance) {
                servo1.setPower(0);
                servo2.setPower(0);
                return false;
            }
            return true;
        }
    }

    // receives the time in milliseconds until the action is considered finished
    public class ClawSampleInteractionAction implements Action {
        private final ClawPowerState wantedPower;
        private final double timeUntilFinished;
        private double startTime;
        private boolean isInitialized = false;

        public ClawSampleInteractionAction(ClawPowerState state, double timeToStop) {
            this.wantedPower = state;
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
        clawInput1 = opMode.hardwareMap.get(AnalogInput.class, "clawInput1");
        clawInput2 = opMode.hardwareMap.get(AnalogInput.class, "clawInput2");

        servo1.setDirection(DcMotorSimple.Direction.FORWARD);
        servo2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public enum ClawPowerState {
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

    public void rotateWheels(ClawPowerState state) {
        wheelRotationState = state;
        servo1.setPower(state.state);
        servo2.setPower(state.state);
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
        return armPosition;
    }

    public ClawPowerState getRotationState() {
        return wheelRotationState;
    }


    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state, double timeUntilFinished) {
        return new ClawSampleInteractionAction(state, timeUntilFinished);
    }

    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state) {
        return new ClawSampleInteractionAction(state, 0);
    }

    // gets in degrees
    public ClawMovementAction setClawMovementAction(double armPosition) {
        return new ClawMovementAction(armPosition);
    }
}
