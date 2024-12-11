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

    // in case the servos act weird
    final double offset1;
    final double offset2;

    // tracks from -∞ - ∞ the rotation of each motor.
    double servo1Rotation = 0;
    double servo2Rotation = 0;

    public class ClawMovementAction implements Action {
        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

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
                setPower(wantedPower);
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

        offset1 = getRotationOfInput(clawInput1);
        offset2 = getRotationOfInput(clawInput2);

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

    private void setPower(double power) {
        servo1.setPower(power);
        servo2.setPower(power);
    }


    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state, double timeUntilFinished) {
        return new ClawSampleInteractionAction(state, timeUntilFinished);
    }

    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state) {
        return new ClawSampleInteractionAction(state, 0);
    }
}
