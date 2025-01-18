/*
--------------------------------------------------------------------------
| This is going to work by internally keeping each servo's rotation in   |
| degrees (0 is starting) and possibly the entire claw's position.       |
| Actions must be used so that the servo actually reaches its position.  |
--------------------------------------------------------------------------
*/

package org.firstinspires.ftc.teamcode.Systems;

import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DifferentialClaws {

    public static final double holdingPower = 0.15;

    CRServo leftClawServo;
    CRServo rightClawServo;
    AnalogInput clawInput1;
    AnalogInput clawInput2;

    double armPosition = 0;
    double lastPosRequest = 0;

    boolean isGoingDown = false;

    // tracks from -∞ - ∞ the rotation of each motor.
    //double leftClawServoRotation = 0;
    //double rightClawServoRotation = 0;

    private double leftClawOldPos;
    private double rightClawOldPos;

    private final double leftClawStart;
    private final double rightClawStart;

    private double trueLeftRotation = 0;
    private double trueRightRotation = 0;

    public PIDController controller;

    public final double p = 0.01, i = 0.01, d = 0.0001;
    public double f = 0.1;

    private double target = 0;

    private ClawPowerState wheelRotationState;

    private double armStartingPosition;

    public class ClawMovementAction implements Action {
//        private double leftClawServoDestination;// in degrees, where 0 is the starting degrees
//        private double rightClawServoDestination;
//
//        private final int directionleftClawServo;
//        private final int directionrightClawServo;
        private boolean isInitialized = false;
//        private final double tolerance = 2.5; //in degrees, how much error can be accepted
        double destination;
        long startTime;
        public ClawMovementAction(double destination) {
//            double pos = lastPosRequest;
//            lastPosRequest = destination;
//            destination -= pos;
//
//            this.leftClawServoDestination = (getleftClawServoRotation() - destination);
//            this.rightClawServoDestination = (getrightClawServoRotation() - destination);
//
//            directionleftClawServo = getleftClawServoRotation() < leftClawServoDestination ? -1 : 1;
//            directionrightClawServo = getrightClawServoRotation() < rightClawServoDestination ? -1 : 1;
//
//            leftClawServoDestination = leftClawServoDestination % 360;
//            rightClawServoDestination = rightClawServoDestination % 360;
            this.destination = destination;
        }


        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setArmTargetPosition(destination);
            updateLeftClawServoRotation();
            updateRightClawServoRotation();
            rotateArm(getPIDArmPower());

            if(!isInitialized){
                isInitialized = true;
                startTime = System.currentTimeMillis();
            }
            return !(System.currentTimeMillis() - startTime >= 750);
        }
    }

    // receives the time in milliseconds until the action is considered finished
    public class ClawSampleInteractionAction implements Action {
        private final double wantedPower;
        private final double timeUntilFinished;
        private double startTime;
        private boolean isInitialized = false;
        private final double bonusRight, bonusLeft;

        public ClawSampleInteractionAction(ClawPowerState state, double timeToStop) {
            this.wantedPower = state.state;
            this.timeUntilFinished = timeToStop;

            bonusLeft = state == ClawPowerState.SPIT? 0 : 2.5*holdingPower;
            bonusRight = state == ClawPowerState.SPIT? 2.5*holdingPower : 0;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInitialized) {
                leftClawServo.setPower(wantedPower + 2.5*holdingPower);
                rightClawServo.setPower(-wantedPower);
                startTime = System.currentTimeMillis();
                isInitialized = true;
            }

            return System.currentTimeMillis() - startTime < timeUntilFinished;
        }
    }

    public class HoldClawAndDropSampleAction implements Action {
        private final double holdingPower = -0.15;
        private final double timeToHold; // time to hold the arm in place until dropping the sample in ms
        private final double timeUntilDropDone; // time until the drop is considered done in ms
        private boolean isHoldInit = false;
        private boolean isDropInit = false;
        private double startTime = 0;


        public HoldClawAndDropSampleAction(double timeToHold, double timeUntilDropDone) {
            this.timeToHold = timeToHold;
            this.timeUntilDropDone = timeUntilDropDone;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isHoldInit) {
                startTime = System.currentTimeMillis();
                rightClawServo.setPower(holdingPower);
                leftClawServo.setPower(holdingPower);
                isHoldInit = true;
            }

            if (!isDropInit && isHoldInit && System.currentTimeMillis() - startTime > timeToHold) {
                rightClawServo.setPower(holdingPower -0.15);
                leftClawServo.setPower(holdingPower +0.15);
                isDropInit = true;
            }

            if (isDropInit && isHoldInit && System.currentTimeMillis() - startTime > timeUntilDropDone) {
                rotateWheels(0);
                return false;
            }
            return true;
        }
    }

    public DifferentialClaws(OpMode opMode) {
        leftClawServo = opMode.hardwareMap.get(CRServo.class, "leftClawServo");
        rightClawServo = opMode.hardwareMap.get(CRServo.class, "rightClawServo");
        clawInput1 = opMode.hardwareMap.get(AnalogInput.class, "clawInput1");
        clawInput2 = opMode.hardwareMap.get(AnalogInput.class, "clawInput2");

        leftClawServo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightClawServo.setDirection(DcMotorSimple.Direction.REVERSE);

        rotateArm(0.01);
        rotateArm(0);

        controller = new PIDController(p, i, d);
        updateLeftClawServoRotation();
        updateRightClawServoRotation();
        rightClawStart = trueRightRotation;
        leftClawStart = trueLeftRotation;
        leftClawOldPos = leftClawStart;
        rightClawOldPos = rightClawStart;
        armStartingPosition = getArmPosition();

    }

    public enum ClawPowerState {
        TAKE_IN(0.75),
        OFF(0),
        SPIT(-0.75);

        public final double state;

        ClawPowerState(double state) {this.state = state;}
    }

    public double getArmPosition() {
        double leftDiff = trueLeftRotation - leftClawStart;
        double rightDiff = trueRightRotation - rightClawStart;

        return Math.abs(((-rightDiff+leftDiff)/2)*1.4);
    }

    public static double getRotationOfInput(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360;
    }

    public void updateLeftClawServoRotation() {
        double currentRotation = getRotationOfInput(clawInput1);
        double diff = currentRotation - leftClawOldPos;

        double newRotationEstimate = 180;
        if(Math.abs(diff) > newRotationEstimate){
            //new rotation occur
            if(diff < 0)
                diff += 360; //add rotation
            else
                diff -= 360; //minus rotation
        }

        leftClawOldPos = currentRotation;
        trueLeftRotation += diff;
    }

    public void updateRightClawServoRotation() {
        double currentRotation = getRotationOfInput(clawInput2);
        double diff = currentRotation - rightClawOldPos;

        double newRotationEstimate = 180;
        if(Math.abs(diff) > newRotationEstimate){
            //new rotation occur
            if(diff < 0)
                diff += 360; //add rotation
            else
                diff -= 360; //minus rotation
        }

        rightClawOldPos = currentRotation;
        trueRightRotation += diff;
    }
    public double getActualArmRotation() {
        return Math.max(getArmPosition() - armStartingPosition, armStartingPosition - getArmPosition());
    }
    public void setF(double f){
        this.f = f;
    }
    public double getPIDArmPower(){
        int armPos = (int)(getActualArmRotation());
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target)) * f;

        return pid + ff;
    }
    public void rotateArm(double power){
        leftClawServo.setPower(power);
        rightClawServo.setPower(power);
    }

    public void rotateWheelsAndHoldSetPower(double holdingPower, double rotatePower){
        leftClawServo.setPower(holdingPower - rotatePower);
        rightClawServo.setPower(holdingPower + rotatePower);
    }

    public void rotateWheels(double state) {
        leftClawServo.setPower(state);
        rightClawServo.setPower(-state);
    }

    public void rotateWheels(ClawPowerState state) {
        wheelRotationState = state;
        leftClawServo.setPower(state.state);
        rightClawServo.setPower(-state.state);
    }

    public void setArmTargetPosition(double pos){
        if(pos >= target){
            isGoingDown = false;
        }else {
            isGoingDown = true;
        }
        target = pos;
    }

    public double getArmTargetPosition(){
        return target;
    }
    public double[] getServoVirtualPosition(){
        return new double[] {trueLeftRotation, trueRightRotation};
    }
    public void setPower(double p1, double p2){
        double sum = p1+p2;
        p1 /= sum;
        p2 /= sum;

        leftClawServo.setPower(p1);
        rightClawServo.setPower(p2);
    }

    public double getleftClawServoRotation() {
        return getRotationOfInput(clawInput1);
    }

    public double getrightClawServoRotation() {
        return getRotationOfInput(clawInput2);
    }

    public double getClawRotation() {
        return armPosition;
    }

    public ClawPowerState getRotationState() {
        return wheelRotationState;
    }


    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state, double timeUntilFinished) {
        return new ClawSampleInteractionAction(state, timeUntilFinished);
    }

    public ClawSampleInteractionAction setClawSampleInteractionAction(ClawPowerState state) {
        return new ClawSampleInteractionAction(state, 0);
    }

    // gets in degrees
//    public ClawMovementAction addClawMovementAction(double armPosition, Telemetry telemetry) {
//        return new ClawMovementAction(armPosition, telemetry);
//    }

    // gets in degrees, adds the given to the current position
    public ClawMovementAction addClawMovementAction(double armPosition) {
        double out_val = this.armPosition + armPosition;
        return clawMovementAction(out_val);
    }

    //gets in degrees, sets the claw's position to the given position
    public ClawMovementAction clawMovementAction(double dest) {
        //double diff = armPosition - this.armPosition;
        //ClawMovementAction action =;
        //this.armPosition = armPosition;
        return new ClawMovementAction(dest);
    }

    public HoldClawAndDropSampleAction test(double timeToHold, double timeToDrop) {
        return new HoldClawAndDropSampleAction(timeToHold, timeToDrop);
    }

//    public ClawMovementAction clawMovementAction(double dest) {
//        return new ClawMovementAction(dest);
//    }
}
