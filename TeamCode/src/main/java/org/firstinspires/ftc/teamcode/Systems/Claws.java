package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;



public class Claws {

    Servo clawServo;
    Servo rotateServo;

    private class ClawAction implements Action {
        private final double clawDestination;
        private final boolean isRotate;

        public ClawAction(double clawDestination) {
            this.clawDestination = clawDestination;
            isRotate = false;
        }

        public ClawAction(double clawDestination, boolean isRotate) {
            this.clawDestination = clawDestination;
            this.isRotate = isRotate;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isRotate) {
                setRotateServo(clawDestination);
            }
            else {
                setClawState(clawDestination);
            }
            return true;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }

    enum ClawState {
        CLAW_CLOSE(1),
        CLAW_OPEN(0),
        ROTATE_CLOSE(1),
        ROTATE_OPEN(0);

        public final int state;

        ClawState(int state) {this.state = state;}
    }

    public Claws(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(Servo.class, "clawServo");
        rotateServo = opMode.hardwareMap.get(Servo.class, "rotateServo");
        setClawState(ClawState.CLAW_OPEN.state);
        setRotateServo(ClawState.ROTATE_OPEN.state);
    }

    // toggles the rotating servo or the claw servo based on isRotate
    public void toggle(boolean isRotate) {
        if (isRotate) {
            toggleRotate();
        }
        else {
            toggleClaw();
        }
    }

    private void toggleClaw() {
        if (clawServo.getPosition() == ClawState.CLAW_OPEN.state) {
            clawServo.setPosition(ClawState.CLAW_CLOSE.state);
        }
        else {
            clawServo.setPosition(ClawState.CLAW_OPEN.state);
        }
    }

    private void toggleRotate() {
        if (rotateServo.getPosition() == ClawState.ROTATE_OPEN.state) {
            rotateServo.setPosition(ClawState.ROTATE_CLOSE.state);
        }
        else {
            rotateServo.setPosition(ClawState.ROTATE_OPEN.state);
        }
    }

    public void setClawState(double state) {
        clawServo.setPosition(state);
    }

    public Action moveClaw(double dest) {
        return new ClawAction(dest);
    }

    public void setRotateServo(double state) {
        rotateServo.setPosition(state);
    }
}
