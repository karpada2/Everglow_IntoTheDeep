package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@Config
@TeleOp(name="ClawPIDFTuning")
public class ClawPIDFTuning extends LinearOpMode {
//
//    private CRServo leftClawServo;
//    private CRServo rightClawServo;
//    private AnalogInput clawInput1;
//    private AnalogInput clawInput2;
//
//    private double leftClawStart;
//    private double rightClawStart;
//
//    private double leftClawOldPos;
//    private double rightClawOldPos;
//
//    private double trueLeftRotation = 0;
//    private double trueRightRotation = 0;
//
//
//    public double getArmPosition() {
//        double leftDiff = trueLeftRotation - leftClawStart;
//        double rightDiff = trueRightRotation - rightClawStart;
//
//        return Math.abs(((-rightDiff+leftDiff)/2)*1.4);
//    }
//
//    public static double getRotationOfInput(AnalogInput input) {
//        return (input.getVoltage() / input.getMaxVoltage()) * 360;
//    }
//
//    public double updateLeftClawServoRotation() {
//        double currentRotation = getRotationOfInput(clawInput1);
//        double diff = currentRotation - leftClawOldPos;
//
//        double newRotationEstimate = 180;
//        if(Math.abs(diff) > newRotationEstimate){
//            //new rotation occur
//            if(diff < 0)
//                diff += 360; //add rotation
//            else
//                diff -= 360; //minus rotation
//        }
//
//        leftClawOldPos = currentRotation;
//        trueLeftRotation += diff;
//        return trueLeftRotation;
////        double curretposLeft = 360 - getRotationOfInput(clawInput1) +leftRotationNum*360;;
////        if ((leftClawOldPos - curretposLeft) >= 300){
////            leftRotationNum +=1;
////        } else if ((leftClawOldPos - curretposLeft) <= -300) {
////            leftRotationNum -=1;
////        }
////
////        leftClawOldPos = 360 - curretposLeft +leftRotationNum*360;
////        return leftClawOldPos;
//    }
//
//    public double updateRightClawServoRotation() {
//        double currentRotation = getRotationOfInput(clawInput2);
//        double diff = currentRotation - rightClawOldPos;
//
//        double newRotationEstimate = 180;
//        if(Math.abs(diff) > newRotationEstimate){
//            //new rotation occur
//            if(diff < 0)
//                diff += 360; //add rotation
//            else
//                diff -= 360; //minus rotation
//        }
//
//        rightClawOldPos = currentRotation;
//        trueRightRotation += diff;
//        return trueRightRotation;
//    }
//
//    public void rotateArm(double power){
//        leftClawServo.setPower(power);
//        rightClawServo.setPower(power);
//    }
//
//    public static double getActualArmRotation(double armStartingPosition, double currArmPosition) {
//        return Math.max(currArmPosition - armStartingPosition, armStartingPosition - currArmPosition);
//    }
//
//    private PIDController controller;
//
//    private double armStartingPosition;
//
//    public static double p = 0.02, i = 0.01, d = 0.0001;
//    public static double f = 0.1;
//
//    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

//        controller = new PIDController(p, i, d);
//        leftClawServo = this.hardwareMap.get(CRServo.class, "leftClawServo");
//        rightClawServo = this.hardwareMap.get(CRServo.class, "rightClawServo");
//        clawInput1 = this.hardwareMap.get(AnalogInput.class, "clawInput1");
//        clawInput2 = this.hardwareMap.get(AnalogInput.class, "clawInput2");
//
//        leftClawServo.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightClawServo.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
        waitForStart();
//
//        rotateArm(0.01);
//        rotateArm(0);
//
//        updateLeftClawServoRotation();
//        updateRightClawServoRotation();
//        rightClawStart = trueRightRotation;
//        leftClawStart = trueLeftRotation;
//        leftClawOldPos = leftClawStart;
//        rightClawOldPos = rightClawStart;
//        armStartingPosition = getArmPosition();

        while (opModeIsActive()) {
//            claws.controller.setPID(p, i, d);
//            int armPos = (int)(getActualArmRotation(armStartingPosition, getArmPosition()));
//            double pid = controller.calculate(armPos, target);
//            double ff = Math.cos(Math.toRadians(target)) * f;
            if(gamepad1.square) {
                claws.updateRightClawServoRotation();
                claws.updateLeftClawServoRotation();
                //double power = pid + ff;

                claws.rotateArm(claws.getPIDArmPower());
            }
            telemetry.addData("pos: ", claws.getActualArmRotation());
            //telemetry.addData("pid", pid);
            //telemetry.addData("ff", ff);
            telemetry.addData("target: ", claws.getArmTargetPosition());
            //telemetry.addData("starting rotation", armStartingPosition);
            telemetry.addData("curr rotation", claws.getArmPosition());
            //telemetry.addData("left claw start", leftClawStart);
            //telemetry.addData("right claw start", rightClawStart);
            //telemetry.addData("left claw rotations", leftRotationNum);
            //telemetry.addData("right claw rotations", leftRotationNum);
            telemetry.addData("right claw", claws.getServoVirtualPosition()[1]);
            telemetry.addData("left claw", claws.getServoVirtualPosition()[0]);
            //telemetry.addData("right claw old", rightClawOldPos);
            //telemetry.addData("left claw old", leftClawOldPos);
           // telemetry.addData("real left pos", getRotationOfInput(clawInput1));

            telemetry.update();

            if (gamepad1.cross) {
                claws.setArmTargetPosition(90);
            }
            else if (gamepad1.circle) {
                claws.setArmTargetPosition(0);
            }
            else if (gamepad1.triangle) {
                Actions.runBlocking(claws.setClawMovementAction(40));
            }
        }
    }
}
