package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Sweeper;

public class DriversOpMode {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1, gamepad2;

    public DriversOpMode(LinearOpMode opMode, Gamepad gamepad1, Gamepad gamepad2){
        this.opMode = opMode;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void run(boolean isBlue){
        Sweeper sweeper = new Sweeper(opMode);
        DifferentialClaws claws = DifferentialClaws.getInstance(opMode);
        MecanumDrive drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(opMode, isBlue);
        Elevators elevators = Elevators.getInstance(opMode);
        elevators.setVerticalPower(0.0);
        boolean isInitialized = false;
        boolean secondery = false;
        ActionControl control = new ActionControl(elevators, claws, colorSensorSystem, drive, gamepad1, gamepad2);
        opMode.waitForStart();
        //LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        //LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");

        double epsilon = 0.4;
        double joystickTolerance = 0.05;
        boolean flagElevatorVerticalDpadDown = true;
        boolean flagElevatorVerticalDpadLeft = true;
        boolean flagElevatorVerticalDpadUp = true;
        boolean flagElevatorVerticalDpadRight = true;

        boolean flagElevatorHorizontalX = true;
        boolean flagElevatorHorizontalTriangle = true;
        boolean flagElevatorHorizontalCircle = true;
        boolean flagElevatorHorizontalSquare = true;

        boolean flagSweeper = true;
        boolean leftBumper = true;
        boolean rightBumper = true;
        double lastPIDPower = 0;
        double virtualClawPose = claws.maxPoint;

        double horElevatorPosition = 0;
        elevators.setHorizontalDestination(Elevators.HorizontalState.HORIZONTAL_RETRACTED.state);

        double startTime = 0;

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;
        boolean isRunPID = false;

        while (opMode.opModeIsActive()) {
            //driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            LinearToExpo(-gamepad1.left_stick_y)*(1.0/Math.pow(4.5, gamepad1.right_trigger)),
                            -gamepad1.left_stick_x*(1.0/Math.pow(4, gamepad1.right_trigger))
                    ),
                    -gamepad1.right_stick_x*(1.0/Math.pow(5, gamepad1.right_trigger))
            ));
            drive.updatePoseEstimate();


            if (gamepad1.circle) {
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_EXTENDED);
            }
            else {
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_RETRACTED);
            }


            claws.updateRightClawServoRotation();
            claws.updateLeftClawServoRotation();

            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                virtualClawPose = DifferentialClaws.ClawPositionState.SPIT_STATE.state-10;
                claws.setArmTargetPosition(virtualClawPose);
                startTime = System.currentTimeMillis();
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;

            if (gamepad2.right_trigger >= 0.4) { //split
                claws.rotateWheels(DifferentialClaws.ClawPowerState.TAKE_IN);
            }
            else if (gamepad2.left_trigger >= 0.4) {
                claws.rotateWheels(gamepad2.left_trigger);
            }
            else if (gamepad2.right_bumper && rightBumper) {
                virtualClawPose = 68;
                claws.setArmTargetPosition(virtualClawPose);
                startTime = System.currentTimeMillis();
            }
            else if (gamepad2.left_bumper && leftBumper) {
                virtualClawPose = 0;
                claws.setArmTargetPosition(virtualClawPose);
//                claws.rotateArm(1);
                startTime = System.currentTimeMillis();
            }
            else {
                //claws.rotateArm(lastPIDPower);
                claws.rotateArm(-gamepad2.left_stick_y); //- Math.cos(Math.toRadians((claws.getActualArmRotation()/claws.maxPoint)*120. - 30.)) * claws.f
            }

            leftBumper = !gamepad2.left_bumper;
            rightBumper = !gamepad2.right_bumper;

            if (Math.abs(gamepad2.right_stick_y) > joystickTolerance) {
                if(horElevatorPosition < 0){
                    horElevatorPosition = 0;
                }else if(horElevatorPosition >= Elevators.HorizontalState.HORIZONTAL_EXTENDED.state){
                    horElevatorPosition =  Elevators.HorizontalState.HORIZONTAL_EXTENDED.state;
                }
                horElevatorPosition += -gamepad2.right_stick_y*40*3;
                elevators.setHorizontalDestination((int)(horElevatorPosition));
            }
            opMode.telemetry.addData("sweeper:", virtualClawPose);
            opMode.telemetry.addData("precieved hor position: ", horElevatorPosition);
            opMode.telemetry.addData("hor position: ", elevators.getHorizontalPosition());
//            telemetry.addData("Control Hub auxillary volts: ", controlHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Expansion Hub auxillary volts: ", expansionHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Control Hub used volts: ", controlHub.getInputVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Expansion Hub used volts: ", expansionHub.getInputVoltage(VoltageUnit.VOLTS));
            opMode.telemetry.update();
            elevators.updateVert();

            if(gamepad2.dpad_down && flagElevatorVerticalDpadDown) {
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_PICKUP.state);
            }
            flagElevatorVerticalDpadDown = !gamepad2.dpad_down;

            if(gamepad2.dpad_left && flagElevatorVerticalDpadLeft){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH.state);
            }
            flagElevatorVerticalDpadLeft = !gamepad2.dpad_left;

            if(gamepad2.dpad_up && flagElevatorVerticalDpadUp){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH.state);
            }
            flagElevatorVerticalDpadUp = !gamepad2.dpad_up;

            if(gamepad2.dpad_right && flagElevatorVerticalDpadRight){
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_HIGH.state+500);
            }
            flagElevatorVerticalDpadRight = !gamepad2.dpad_right;

            opMode.telemetry.addData("vert pos:", elevators.getVerticalCurrentPosition());

            if(isInitialized && secondery)
                elevators.updateVert();
            colorSensorSystem.updateAlert();

            if(gamepad2.square && !isInitialized){
                elevators.setVerticalDestination(-Elevators.VerticalState.VERTICAL_HIGH.state);
                isInitialized = true;
                flagElevatorHorizontalSquare = false;
            }

            if(gamepad2.square && isInitialized && flagElevatorHorizontalSquare){
                elevators.resetVert();
                elevators.setVerticalDestination(0);
                secondery = true;
            }
            flagElevatorHorizontalSquare = !gamepad2.square;

            if(System.currentTimeMillis()-startTime <3000) {
                if(claws.getArmTargetPosition() == 0)
                {
                    if(claws.getActualArmRotation() >58)
                        lastPIDPower = 0.2;
                    else
                        lastPIDPower = -Math.cos(Math.toRadians(
                                ((int)(claws.getActualArmRotation())/DifferentialClaws.maxPoint)*120. - 30.)) * -0.05;
                }
                else
                    lastPIDPower = claws.getPIDArmPower();

                claws.rotateArm(lastPIDPower);
                if ((claws.getActualArmRotation() <= 5 && claws.getArmTargetPosition() ==0)
                        || (claws.getActualArmRotation() >= 67 && claws.getArmTargetPosition() == 68))
                    lastPIDPower = 0;
            }
        }

    }
    public static double LinearToExpo(double input) {
        if (input >= 0) return input*input;
        else return -input*input;
    }
}
