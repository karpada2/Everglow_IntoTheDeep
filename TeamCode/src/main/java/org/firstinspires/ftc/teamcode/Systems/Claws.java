package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;


public class Claws {

    CRServo clawServo;

    private class ClawAction implements Action {

        public ClawAction() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            toggleServo();
            return true;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }

    public enum ClawState {
        OFF(0),
        ON(0.5);

        public final double state;

        ClawState(double state) {
            this.state = state;
        }
    }

    public Claws(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(CRServo.class, "clawServo");
        clawServo.setPower(ClawState.OFF.state);
    }

    public boolean isActive() {
        return clawServo.getPower() == ClawState.ON.state;
    }

    public void setServo(ClawState state) {
        clawServo.setPower(state.state);
    }

    public void toggleServo() {
        if (isActive()) {
            setServo(ClawState.OFF);
        }
        else {
            setServo(ClawState.ON);
        }
    }
}
