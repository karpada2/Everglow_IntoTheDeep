package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;



public class Claws {



    private class ClawAction implements Action {
        private double clawDestination;

        public ClawAction(double clawDestination) {
            this.clawDestination = clawDestination;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setClawState(clawDestination);
            return true;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }
    }

    enum ClawState {
        CLOSE(1),
        OPEN(0);

        public final int state;

        ClawState(int state) {this.state = state;}
    }

    Servo clawServo;
    Servo rotateServo;

    public Claws(OpMode opMode) {
        clawServo = opMode.hardwareMap.get(Servo.class, "clawServo");
        rotateServo = opMode.hardwareMap.get(Servo.class, "rotateServo");
        setClawState(ClawState.OPEN.state);
    }

    public void toggleClaw() {
        if (clawServo.getPosition() == ClawState.OPEN.state) {
            clawServo.setPosition(ClawState.CLOSE.state);
        }
        else {
            clawServo.setPosition(ClawState.CLOSE.state);
        }
    }

    public void setClawState(double state) {
        clawServo.setPosition(state);
    }

    public Action moveClaw(double dest) {
        return new ClawAction(dest);
    }

    // testing git pushes
}
