package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Claws {

    DcMotorEx clawMotor;

    private class ClawAction implements Action {

        public ClawAction() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            toggleMotor();
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
        clawMotor = opMode.hardwareMap.get(DcMotorEx.class, "clawMotor");
        clawMotor.setPower(ClawState.OFF.state);
        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isActive() {
        return clawMotor.getPower() == ClawState.ON.state;
    }

    public void setMotor(ClawState state) {
        clawMotor.setPower(state.state);
    }

    public void toggleMotor() {
        if (isActive()) {
            setMotor(ClawState.OFF);
        }
        else {
            setMotor(ClawState.ON);
        }
    }
}
