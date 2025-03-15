package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.Systems.DifferentialClaws.maxPoint;
import static org.firstinspires.ftc.teamcode.tuning.ClawPIDFTuning.f;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Sweeper;

public class BestOpMode{
    public static double linearToExpo(double input) {
        return (input >= 0) ? input*input : -input*input;
    }
    public BestOpMode(LinearOpMode opMode, Gamepad gamepad1, Gamepad gamepad2) {
        this.opMode = opMode;
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);
        this.telemetry = opMode.telemetry;
    }
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final GamepadEx gamepad1;
    private final GamepadEx gamepad2;
    boolean isPIDF_Active = false;

    public void run(boolean isBlue) {
        Sweeper sweeper = new Sweeper(opMode);
        DifferentialClaws claws = DifferentialClaws.getInstance(opMode);
        MecanumDrive drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(opMode, isBlue);

        Elevators elevators = Elevators.getInstance(opMode);
        elevators.setVerticalPower(0.0);

        ActionControl actionControl = new ActionControl(elevators, claws, colorSensorSystem, drive, sweeper, gamepad1, gamepad2);

        double joystickTolerance = 0.05;

        double horElevatorPosition = 0;
        double targetArmPosition = claws.getActualArmRotation();

        elevators.setHorizontalDestination(Elevators.HorizontalState.HORIZONTAL_RETRACTED.state);

        opMode.waitForStart();

        double startTime = System.currentTimeMillis();
        int loopsDone = 0;
        double timeSinceStartSecs = (System.currentTimeMillis() - startTime)/1000.0;

        while (opMode.opModeIsActive()) {
            loopsDone++;
            timeSinceStartSecs = (System.currentTimeMillis() - startTime)/1000.0;

            gamepad1.readButtons();
            gamepad2.readButtons();

            claws.updateLeftClawServoRotation();
            claws.updateRightClawServoRotation();

            colorSensorSystem.updateAlert();

            //driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            linearToExpo(gamepad1.getLeftY())*(1.0/Math.pow(4.5, gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))),
                            -gamepad1.getLeftX()*(1.0/Math.pow(4, gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
                    ),
                    -gamepad1.getRightX()*(1.0/Math.pow(5, gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
            ));
            drive.updatePoseEstimate();


            if (gamepad1.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_EXTENDED);
            }
            else {
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_RETRACTED);
            }



            if (gamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_OPMODE_HIGH.state);
            }
            else if (gamepad2.wasJustPressed(GamepadKeys.Button.X)) {
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH.state);
            }
            else if (gamepad2.wasJustPressed(GamepadKeys.Button.A)) {
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_PICKUP.state);
            }

            if (Math.abs(gamepad2.getRightY()) > joystickTolerance) {
                horElevatorPosition += -gamepad2.getRightY()*0.015 * (gamepad2.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON) ? 1.0/3.0 : 1);
                if(horElevatorPosition < Elevators.HorizontalState.HORIZONTAL_RETRACTED.state){
                    horElevatorPosition = Elevators.HorizontalState.HORIZONTAL_RETRACTED.state;
                }else if(horElevatorPosition >= Elevators.HorizontalState.HORIZONTAL_EXTENDED.state){
                    horElevatorPosition =  Elevators.HorizontalState.HORIZONTAL_EXTENDED.state;
                }
            }
            elevators.setHorizontalDestination(horElevatorPosition);


            if (gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.4) { //split
                claws.rotateWheels(DifferentialClaws.ClawPowerState.TAKE_IN);
            }
            else if (gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.4) {
                claws.rotateWheels(-gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            }
            else if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                targetArmPosition = DifferentialClaws.ClawPositionState.MIN.state;
            }
            else if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                targetArmPosition = DifferentialClaws.ClawPositionState.SPIT_STATE.state;
            }
            else if (gamepad2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                targetArmPosition = DifferentialClaws.ClawPositionState.MAX.state;
            }

            isPIDF_Active = !(gamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.4 || gamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.4);

            targetArmPosition += (-gamepad2.getLeftY()/2.5) * (gamepad2.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) ? 1.0/2.0 : 1);


            if (targetArmPosition > maxPoint) {
                targetArmPosition = maxPoint;
            }
            else if (targetArmPosition < 0) {
                targetArmPosition = 0;
            }

            if(isPIDF_Active) {
                claws.setArmTargetPosition(targetArmPosition);

                claws.rotateArm(claws.getPIDArmPower());
            }


            if (gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                Actions.runBlocking(actionControl.dropHigh());
            }
            else if (gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                Actions.runBlocking(new SequentialAction(actionControl.splineToDropLine(), actionControl.dropHighAndToPlace()));
            }

            telemetry.addData("loops done", loopsDone);
            telemetry.addData("time since start", timeSinceStartSecs);
            telemetry.addData("loops per second avg", loopsDone/timeSinceStartSecs);
            telemetry.addData("target pos", targetArmPosition);
            telemetry.addData("claw pos", claws.getActualArmRotation());
            telemetry.update();
        }
    }
}
