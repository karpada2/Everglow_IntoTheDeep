package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevators {
    final int epsilon = 5;

    DcMotorEx rightVert;
    DcMotorEx leftVert;
    Servo rightHor;
    Servo leftHor;

    int verticalDestination;

    // sets the vertical elevator to the specified position
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

    // sets the horizontal elevator to the specified position
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
            setHorizontalPosition(destination);
            return true;
        }
    }

    // Vertical min is lowest possible, max is highest possible, low and high are terms for the baskets
    public enum VerticalState {
        VERTICAL_MIN(0),
        VERTICAL_PICKUP(0),
        VERTICAL_HURDLE(480),
        VERTICAL_LOW(3070),
        VERTICAL_HIGH(4243),
        VERTICAL_MAX(4243);


        public final int state;

        VerticalState(int state) {
            this.state = state;
        }
    }

    public enum HorizontalState{
        HORIZONTAL_RETRACTED(0),
        HORIZONTAL_HALFWAY(0.55),
        HORIZONTAL_EXTENDED(0.685),
        HORIZONTAL_DROP(0.5),
        HORIZONTAL_MAX(1.0);

        public final double state;

        HorizontalState(double state) {
            this.state = state;
        }
    }

    public Elevators(OpMode opMode) {
        rightVert = opMode.hardwareMap.get(DcMotorEx.class, "rightVert");
        leftVert = opMode.hardwareMap.get(DcMotorEx.class, "leftVert");
        rightHor = opMode.hardwareMap.get(Servo.class, "rightHor");
        leftHor = opMode.hardwareMap.get(Servo.class, "leftHor");

        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVert.setDirection(DcMotorSimple.Direction.FORWARD);

        setVerticalDestination(VerticalState.VERTICAL_MIN.state);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftHor.setDirection(Servo.Direction.REVERSE);
        rightHor.setDirection(Servo.Direction.FORWARD);

        setHorizontalPosition(HorizontalState.HORIZONTAL_RETRACTED.state);

    }

    public int getVerticalDestination() {
        return verticalDestination;
    }

    public int getVerticalCurrentPosition() {
        return rightVert.getCurrentPosition();
    }

    // sets the destination of the vertical motors to the specified number of ticks
    public void setVerticalDestination(int destination) {
        rightVert.setTargetPosition(destination);
        leftVert.setTargetPosition(destination);
        verticalDestination = destination;
    }

    public double getHorizontalState() {
        return rightHor.getPosition();
    }


    public void setHorizontalPosition(double position) {
        rightHor.setPosition(position);
        leftHor.setPosition(position);
    }

    // toggles the horizontal elevator between being
    public void toggleHorizontal() {
        if (getHorizontalState() == HorizontalState.HORIZONTAL_EXTENDED.state) {
            setHorizontalPosition(HorizontalState.HORIZONTAL_RETRACTED.state);
        }
        else {
            setHorizontalPosition(HorizontalState.HORIZONTAL_EXTENDED.state);
        }
    }

    // checks whether the elevator is close enough (+- epsilon) to it's destination
    public boolean isElevatorInDestination() {
        if (getVerticalCurrentPosition() < getVerticalDestination()) {
            return getVerticalCurrentPosition() >= getVerticalDestination() - epsilon;
        }
        return getVerticalCurrentPosition() <= getVerticalDestination() + epsilon;
    }

    public void setVerticalPower(double power){
        rightVert.setPower(power);
        leftVert.setPower(power);
    }

    public Action vertMoveTo(VerticalState state) {
        return new Elevators.VerticalElevatorAction(state.state);
    }

    public Action horMoveTo(HorizontalState state) {
        return new Elevators.HorizontalElevatorAction(state.state);
    }
}
