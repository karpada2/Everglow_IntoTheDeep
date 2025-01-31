package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Elevators{
    final int epsilon = 100;


    DcMotorEx rightVert;
    DcMotorEx leftVert;
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

            if (!isInitialized) {
                setVerticalDestination(this.destination);
                isInitialized = true;
            }

            updateVert();
            
            return !isElevatorInDestination();
        }
    }


    /*
    -------------------------------------------------------------------
    | stepSize and tolerance need to be tuned so it feels good to use |
    -------------------------------------------------------------------
     */
    // moves the horizontal elevators to destination, and is considered finished when they reach the destination
    public class MotorHorizontalElevatorAction implements Action {
        private final int destination;
        private boolean isInitialized = false;

        public MotorHorizontalElevatorAction(MotorHorizontalState state) {
            //motorSetHorizontalDestination(state);
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
                motorSetHorizontalDestination(this.destination);
                isInitialized = true;
            }


            return !motorIsHorizontalInDestination();
        }
    }



    // Vertical min is lowest possible, max is highest possible, low and high are terms for the baskets
    public enum VerticalState {
        VERTICAL_MIN(0),
        VERTICAL_PICKUP(0),
        VERTICAL_HURDLE(970),
        VERTICAL_SPECIMEN_HIGH(4553),
        VERTICAL_LOW(7643),
        VERTICAL_HIGH(11000),
        VERTICAL_MAX(11448);


        public final int state;

        VerticalState(int state) {
            this.state = state;
        }
    }

    public enum MotorHorizontalState{
        HORIZONTAL_RETRACTED(0),
        HORIZONTAL_HALFWAY(400),
        HORIZONTAL_EXTENDED(1440);

        public final int state;

        MotorHorizontalState(int state) {
            this.state = state;
        }
    }

    public Elevators(OpMode opMode, boolean isToRestart) {
        rightVert = opMode.hardwareMap.get(DcMotorEx.class, "rightVert");
        leftVert = opMode.hardwareMap.get(DcMotorEx.class, "leftVert");
        horMotor = opMode.hardwareMap.get(DcMotorEx.class, "motorHor");

        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVert.setDirection(DcMotorSimple.Direction.FORWARD);

        if(isToRestart)
            setVerticalDestination(VerticalState.VERTICAL_MIN.state);
        else
            setVerticalDestination(leftVert.getCurrentPosition());
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if(isToRestart) {
            rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        leftHor.setDirection(Servo.Direction.REVERSE);
//        rightHor.setDirection(Servo.Direction.FORWARD);

        horMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSetHorizontalDestination(MotorHorizontalState.HORIZONTAL_RETRACTED);
        horMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(isToRestart)
            horMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

//        if(destination<VerticalState.VERTICAL_MIN.state || destination>VerticalState.VERTICAL_MAX.state){
//            return;
//        }

        double eps = 60;
        if (Math.abs(destination-getVerticalCurrentPosition())<=60 && destination == 0) {
            //setVerticalPower(0);
        }
        else {
            setVerticalPower(0.8);
        }
        rightVert.setTargetPosition(destination);
        leftVert.setTargetPosition(destination);
        verticalDestination = destination;
    }

    public int getVertDestination(){
        return verticalDestination;
    }

    public void updateVert(){
        if (Math.abs(verticalDestination-getVerticalCurrentPosition())<=50 && verticalDestination == 0) {
            setVerticalPower(0);
            double innerEps = 10;
            if(Math.abs(verticalDestination-getVerticalCurrentPosition())>= innerEps) {
                rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else if(Math.abs(verticalDestination-getVerticalCurrentPosition())<=epsilon){
            setVerticalPower(0.6);
        }
        else {
            setVerticalPower(1);
        }
    }

    // checks whether the elevator is close enough (+- epsilon) to it's destination
    public boolean isElevatorInDestination() {
//        if (getVerticalCurrentPosition() < getVerticalDestination()) {
//            return getVerticalCurrentPosition() >= getVerticalDestination() - epsilon;
//        }
        return Math.abs(getVerticalCurrentPosition() - getVerticalDestination()) <= epsilon;
    }

    public void setVerticalPower(double power){
        rightVert.setPower(power);
        leftVert.setPower(power);
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

    public void motorSetHorizontalDestination(int destination) {
        double eps = 12;
        if(destination>=MotorHorizontalState.HORIZONTAL_RETRACTED.state && destination<=MotorHorizontalState.HORIZONTAL_EXTENDED.state) {
            if (Math.abs(destination - motorGetHorizontalPosition()) < eps) {
                motorSetHorizontalPower(0);
            } else {
                motorSetHorizontalPower(0.8);
            }
            horMotor.setTargetPosition(destination);
        }
    }

    public void motorSetHorizontalPower(double power) {
        horMotor.setPower(power);
    }

    public Action getHorizontalAction(MotorHorizontalState state){
        return new MotorHorizontalElevatorAction(state);
    }

    public Action getVerticalAction(VerticalState state){
        return new VerticalElevatorAction(state.state);

    }
    public VerticalElevatorAction setVerticalElevatorAction(VerticalState targetState) {
        return new VerticalElevatorAction(targetState.state);
    }

    public void resetVert(){
        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setPower(0);
        rightVert.setPower(0);
    }

    public MotorHorizontalElevatorAction setMotorHorizontalElevatorAction(MotorHorizontalState destinationState) {
        return new MotorHorizontalElevatorAction(destinationState);
    }
    public double getHorizontalPosition(){
        return horMotor.getCurrentPosition();
    }
    public void setVertDest(int dest){
        rightVert.setTargetPosition(dest);
        leftVert.setTargetPosition(dest);
        rightVert.setPower(0.65);
        leftVert.setPower(0.65);
    }
}
