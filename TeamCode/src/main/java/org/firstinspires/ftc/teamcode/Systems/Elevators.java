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

public class Elevators{
    final int epsilon = 5;

    DcMotorEx rightVert;
    DcMotorEx leftVert;
    Servo rightHor;
    Servo leftHor;
    DcMotorEx horMotor;

    int verticalDestination;
    int motorHorizontalDestination;

    // sets the vertical elevator to the specified position
    public class VerticalElevatorAction implements Action {
        private final int destination;
        private boolean isInitialized = false;

        public VerticalElevatorAction(int destination) {
            this.destination = destination;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (leftVert.getPower() == 0.0) {
                setVerticalPower(0.8);
            }
            if (!isInitialized) {
                setVerticalDestination(this.destination);
                isInitialized = true;
            }

            return !isElevatorInDestination();
        }
    }


    /*
    -------------------------------------------------------------------
    | stepSize and tolerance need to be tuned so it feels good to use |
    -------------------------------------------------------------------
     */
    // moves the horizontal elevators to destination, and is considered finished when they reach the destination
    public class HorizontalElevatorAction implements Action {
        private final double destination;
        private double position;
        private final double stepSize = 0.0003;
        private final double directionToMove;
        private final double tolerance = 0.01;

        public HorizontalElevatorAction(double destination) {
            this.destination = destination;
            this.position = leftHor.getPosition();
            directionToMove = this.destination > this.position ? 1 : -1;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            this.position += stepSize * directionToMove;
            setHorizontalPosition(this.position);

            return Math.abs(this.position - this.destination) >= tolerance;
        }
    }

    public class MotorHorizontalElevatorAction implements Action {
        private final int destination;
        private boolean isInitialized = false;

        public MotorHorizontalElevatorAction(MotorHorizontalState state) {
            motorSetHorizontalDestination(state);
            this.destination = state.state;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (horMotor.getPower() == 0) {
                motorSetHorizontalPower(0.8);
            }
            if (!isInitialized) {
                setVerticalDestination(this.destination);
                isInitialized = true;
            }

            return !motorIsHorizontalInDestination();
        }
    }



    // Vertical min is lowest possible, max is highest possible, low and high are terms for the baskets
    public enum VerticalState {
        VERTICAL_MIN(0),
        VERTICAL_PICKUP(0),
        VERTICAL_HURDLE(720),
        VERTICAL_LOW(3070),
        VERTICAL_HIGH(4243),
        VERTICAL_MAX(4243);


        public final int state;

        VerticalState(int state) {
            this.state = state;
        }
    }

    public enum HorizontalState{
        HORIZONTAL_RETRACTED(0.36),
        HORIZONTAL_HALFWAY(0.68),
        HORIZONTAL_EXTENDED(0.99),
//        HORIZONTAL_DROP(0.5),
        HORIZONTAL_MAX(1.0);

        public final double state;

        HorizontalState(double state) {
            this.state = state;
        }
    }

    public enum MotorHorizontalState{
        HORIZONTAL_RETRACTED(0),
        HORIZONTAL_HALFWAY(100),
        HORIZONTAL_EXTENDED(200);

        public final int state;

        MotorHorizontalState(int state) {
            this.state = state;
        }
    }

    public Elevators(OpMode opMode) {
        rightVert = opMode.hardwareMap.get(DcMotorEx.class, "rightVert");
        leftVert = opMode.hardwareMap.get(DcMotorEx.class, "leftVert");
        rightHor = opMode.hardwareMap.get(Servo.class, "rightHor");
        leftHor = opMode.hardwareMap.get(Servo.class, "leftHor");
        horMotor = opMode.hardwareMap.get(DcMotorEx.class, "motorHor");

        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVert.setDirection(DcMotorSimple.Direction.FORWARD);

        setVerticalDestination(VerticalState.VERTICAL_MIN.state);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        leftHor.setDirection(Servo.Direction.REVERSE);
//        rightHor.setDirection(Servo.Direction.FORWARD);

        horMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSetHorizontalDestination(MotorHorizontalState.HORIZONTAL_RETRACTED);
        horMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//TODO: Run to position

//        setHorizontalPosition(HorizontalState.HORIZONTAL_RETRACTED.state);


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

    public double getLeftHorPos() {
        return leftHor.getPosition();
    }

    public double getRightHorPos() {
        return rightHor.getPosition();
    }

    public int motorGetHorizontalPosition() {
        return horMotor.getCurrentPosition();
    }

    public int motorGetHorizontalDestination() {
        return motorHorizontalDestination;
    }

    public boolean motorIsHorizontalInDestination() {
        return Math.abs(motorGetHorizontalPosition() - motorGetHorizontalDestination()) < epsilon;
    }

    public void motorSetHorizontalDestination(MotorHorizontalState state) {
        this.motorHorizontalDestination = state.state;
        horMotor.setTargetPosition(state.state);
    }

    public void motorSetHorizontalPower(double power) {
        horMotor.setPower(power);
    }

    public Action getHorizontalAction(HorizontalState state){
        return new HorizontalElevatorAction(state.state);
    }

    public Action getVerticalAction(VerticalState state){
        return new VerticalElevatorAction(state.state);

    }
    public VerticalElevatorAction setVerticalElevatorAction(VerticalState targetState) {
        return new VerticalElevatorAction(targetState.state);
    }

    public HorizontalElevatorAction setHorizontalElevatorAction(double horizontalTarget) {
        return new HorizontalElevatorAction(horizontalTarget);
    }

    public MotorHorizontalElevatorAction setMotorHorizontalElevatorAction(MotorHorizontalState destinationState) {
        return new MotorHorizontalElevatorAction(destinationState);
    }
}
