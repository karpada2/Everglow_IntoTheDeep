package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevators {
    final int epsilon = 5;

    DcMotorEx verticalMotor1;
    DcMotorEx verticalMotor2;
    Servo horizontalServo1;
    Servo horizontalServo2;

    int verticalDestination;

    public class VerticalElevatorAction implements Action {
        private final int destination;

        public VerticalElevatorAction(int destination) {
            this.destination = destination;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setVerticalDestination(destination);
            return true;
        }
    }

    public class HorizontalElevatorAction implements Action {
        private final double destination;

        public HorizontalElevatorAction(double destination) {
            this.destination = destination;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setHorizontalState(destination);
            return true;
        }
    }

    // Vertical min is lowest possible, max is highest possible, low and high are terms for the baskets
    public enum ElevatorState {
        VERTICAL_MIN(0),
        VERTICAL_PICKUP(1),
        VERTICAL_LOW(2),
        VERTICAL_HIGH(3),
        VERTICAL_MAX(4),
        HORIZONTAL_RETRACTED(5),
        HORIZONTAL_EXTENDED(6);

        public final double state;

        ElevatorState(double state) {
            this.state = state;
        }
    }

    public Elevators(OpMode opMode) {
        verticalMotor1 = opMode.hardwareMap.get(DcMotorEx.class, "verticalMotor1");
        verticalMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "verticalMotor2");
        horizontalServo1 = opMode.hardwareMap.get(Servo.class, "horizontalServo1");
        horizontalServo2 = opMode.hardwareMap.get(Servo.class, "horizontalServo2");

        verticalMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setVerticalDestination((int) ElevatorState.VERTICAL_MIN.state);

        setHorizontalState(ElevatorState.HORIZONTAL_RETRACTED.state);
    }

    public int getVerticalDestination() {
        return verticalDestination;
    }

    public int getVerticalCurrentPosition() {
        return verticalMotor1.getCurrentPosition();
    }

    public void setVerticalDestination(int destination) {
        verticalMotor1.setTargetPosition(destination);
        verticalMotor2.setTargetPosition(destination);
        verticalDestination = destination;
    }

    public double getHorizontalState() {
        return horizontalServo1.getPosition();
    }

    public void setHorizontalState(double destination) {
        horizontalServo1.setPosition(destination);
        horizontalServo2.setPosition(destination);
    }

    public void toggleHorizontal() {
        if (getHorizontalState() == ElevatorState.HORIZONTAL_RETRACTED.state) {
            setHorizontalState(ElevatorState.HORIZONTAL_RETRACTED.state);
        }
        else {
            setHorizontalState(ElevatorState.HORIZONTAL_EXTENDED.state);
        }
    }

    public boolean isElevatorInDestination() {
        if (getVerticalCurrentPosition() < getVerticalDestination()) {
            return getVerticalCurrentPosition() >= getVerticalDestination() - epsilon;
        }
        return getVerticalCurrentPosition() <= getVerticalDestination() + epsilon;
    }
}
