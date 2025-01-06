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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DifferentialClaws {

    public static final double holdingPower = 0.1;

    public static double getRotationOfInput(AnalogInput input) {
        return (input.getVoltage() / input.getMaxVoltage()) * 360;
    }

    CRServo leftClawServo;
    CRServo rightClawServo;
    AnalogInput clawInput1;
    AnalogInput clawInput2;

    double armPosition = 0;

    // tracks from -∞ - ∞ the rotation of each motor.
    double leftClawServoRotation = 0;
    double rightClawServoRotation = 0;

    private ClawPowerState wheelRotationState;

    public class ClawMovementAction implements Action {
        private double leftClawServoDestination;// in degrees, where 0 is the starting degrees
        private double rightClawServoDestination;

        private final int directionleftClawServo;
        private final int directionrightClawServo;
        private boolean isInitialized = false;

        private Telemetry telemetry;

        private final double power = 0.16;
        private final double tolerance = 5; //in degrees, how much error can be accepted
        private final int goUp;
        public ClawMovementAction(double destination) {
            this.leftClawServoDestination = (getleftClawServoRotation() + destination);
            this.rightClawServoDestination = (getrightClawServoRotation() + destination);

            goUp =  destination > 0? 1 : -1;

            directionleftClawServo = getleftClawServoRotation() < leftClawServoDestination ? -1 : 1;
            directionrightClawServo = getrightClawServoRotation() < rightClawServoDestination ? -1 : 1;

            leftClawServoDestination = leftClawServoDestination % 360;
            rightClawServoDestination = rightClawServoDestination % 360;

        }

        public ClawMovementAction(double destination, Telemetry telemetry) {
            this.leftClawServoDestination = (getleftClawServoRotation() - destination);
            this.rightClawServoDestination = (getrightClawServoRotation() - destination);

            goUp =  destination > 0? 1 : -1;

            directionleftClawServo = getleftClawServoRotation() < leftClawServoDestination ? -1 : 1;
            directionrightClawServo = getrightClawServoRotation() < rightClawServoDestination ? -1 : 1;

            leftClawServoDestination = leftClawServoDestination % 360;
            rightClawServoDestination = rightClawServoDestination % 360;

            this.telemetry = telemetry;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("in action left servo rotation: ", getleftClawServoRotation());
            telemetry.addData("in action right servo rotation: ", getrightClawServoRotation());
            telemetry.addData("left claw condition: ", abs(getleftClawServoRotation() - leftClawServoDestination) < tolerance);
            telemetry.addData("right claw condition: ", abs(getleftClawServoRotation() - leftClawServoDestination) < tolerance);

            telemetry.update();

            if (!isInitialized) {
                leftClawServo.setPower(power * directionleftClawServo);
                rightClawServo.setPower(power * directionrightClawServo);
                isInitialized = true;
            }

            if (abs(getleftClawServoRotation() - leftClawServoDestination) < tolerance || abs(getleftClawServoRotation() - leftClawServoDestination) < tolerance) {
                leftClawServo.setPower(holdingPower);
                rightClawServo.setPower(holdingPower);
                return false;
            }
            return true;
        }
    }

    // receives the time in milliseconds until the action is considered finished
    public class ClawSampleInteractionAction implements Action {
        private final double wantedPower;
        private final double timeUntilFinished;
        private double startTime;
        private boolean isInitialized = false;

        public ClawSampleInteractionAction(ClawPowerState state, double timeToStop) {
            this.wantedPower = state.state;
            this.timeUntilFinished = timeToStop;
        }

        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!isInitialized) {
                leftClawServo.setPower(wantedPower + holdingPower);
                rightClawServo.setPower(-wantedPower + holdingPower);
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
    }

    public enum ClawPowerState {
        TAKE_IN(0.75),
        OFF(0),
        SPIT(-0.75);

        public final double state;

        ClawPowerState(double state) {this.state = state;}
    }

    public void rotateArm(double power){
        power /= 2;
        leftClawServo.setPower(power);
        rightClawServo.setPower(power);
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
    public ClawMovementAction setClawMovementAction(double armPosition, Telemetry telemetry) {
        return new ClawMovementAction(armPosition, telemetry);
    }

    // gets in degrees
    public ClawMovementAction setClawMovementAction(double armPosition) {
        return new ClawMovementAction(armPosition);
    }

    public HoldClawAndDropSampleAction test(double timeToHold, double timeToDrop) {
        return new HoldClawAndDropSampleAction(timeToHold, timeToDrop);
    }
}
