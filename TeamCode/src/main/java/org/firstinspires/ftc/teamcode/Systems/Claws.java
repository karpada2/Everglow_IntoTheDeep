package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;


public class Claws {

    CRServo claw;

    public class ClawAction implements Action {
        private final ClawState targetState;
        private final double startTime;
        private final double stopTime;
        private boolean isInitialized = false;

        private ClawAction(ClawState targetState, double timeToFinish) {
            this.targetState = targetState;
            this.startTime = System.currentTimeMillis();
            this.stopTime = timeToFinish;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInitialized) {
                setState(targetState);
                isInitialized = true;
            }

            return System.currentTimeMillis() - startTime >= stopTime;
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

    public Claws(OpMode opMode) {
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

    // receives what to set the claw to and the time in milliseconds until the action is considered finished
    public ClawAction setClawAction(ClawState targetState, double timeUntilFinished) {
        return new ClawAction(targetState, timeUntilFinished);
    }

    // receives what to set the claw to and considers the action to be immediately finished
    public ClawAction setClawAction(ClawState targetState) {
        return new ClawAction(targetState, 0);
    }
}
