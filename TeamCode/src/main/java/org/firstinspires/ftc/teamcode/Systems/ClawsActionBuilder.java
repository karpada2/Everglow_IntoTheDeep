package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;


public class ClawsActionBuilder {

    private CRServo claw;

    // takes in a boolean, and takes in a sample if it is true, turns off otherwise
    private class ClawTakeInAction implements Action {
        boolean turnOn = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (turnOn) {
                setState(ClawState.OFF);
                turnOn = false;
            } else {
                setState(ClawState.TAKE_IN);
                turnOn = true;
            }
            return false;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }
    public Action clawTakeInAction() {
        return new ClawTakeInAction();
    }

    // takes in a boolean, and spits out the sample if it is true, turns off otherwise
    private class ClawSpitAction implements Action {
        boolean turnOn = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (turnOn) {
                setState(ClawState.OFF);
                turnOn = false;
            } else {
                setState(ClawState.SPIT);
                turnOn = true;
            }
            return false;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }
    public Action clawSpitAction() {
        return new ClawSpitAction();
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
