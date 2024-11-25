package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;


public class ClawsActionBuilder {

    CRServo claw;

    // takes in a boolean, and takes in a sample if it is true, turns off otherwise
    private class ClawTakeInAction implements Action {
        boolean turnOn;

        public ClawTakeInAction(boolean turnOn) {
            this.turnOn = turnOn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (turnOn) {
                setState(ClawState.TAKE_IN);
            } else {
                setState(ClawState.OFF);
            }
            return true;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }

    // takes in a boolean, and spits out the sample if it is true, turns off otherwise
    private class ClawSpitAction implements Action {
        boolean turnOn;

        public ClawSpitAction(boolean turnOn) {
            this.turnOn = turnOn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (turnOn) {
                setState(ClawState.SPIT);
            } else {
                setState(ClawState.OFF);
            }
            return true;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }

    public enum ClawState {
        SPIT(-0.5),
        OFF(0),
        TAKE_IN(0.5);

        public final double state;

        ClawState(double state) {
            this.state = state;
        }
    }

    public ClawsActionBuilder(OpMode opMode) {
        claw = opMode.hardwareMap.get(CRServo.class, "claw");
        claw.setPower(ClawState.OFF.state);
    }

    public ClawState getState() {
        if (claw.getPower() == ClawState.SPIT.state) {
            return ClawState.SPIT;
        }
        else if (claw.getPower() == ClawState.TAKE_IN.state) {
            return ClawState.TAKE_IN;
        }
        else {
            return ClawState.OFF;
        }
    }

    // sets the servo to the needed power level in the enum
    public void setState(ClawState state) {
        claw.setPower(state.state);
    }
}
