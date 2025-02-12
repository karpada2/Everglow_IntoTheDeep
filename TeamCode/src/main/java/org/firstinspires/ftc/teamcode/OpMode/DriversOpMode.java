package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

public class DriversOpMode {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1, gamepad2;

    public DriversOpMode(LinearOpMode opMode, Gamepad gamepad1, Gamepad gamepad2){
        this.opMode = opMode;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void run(boolean isBlue){
        DifferentialClaws claws = new DifferentialClaws(opMode);
        MecanumDrive drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));

        Elevators elevators = new Elevators(opMode);
        elevators.setVerticalPower(0.0);
        boolean isInitialized = false;
        boolean secondery = false;

        opMode.waitForStart();
        //LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        //LynxModule expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(opMode, isBlue);

        elevators.motorSetHorizontalPower(1);

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

        boolean flagClawTakeIn = true;
        boolean ClawState = true;
        boolean flagClawSpit = true;
        double lastPIDPower = 0;
        double virtualClawPose = claws.maxPoint;

        double horElevatorPosition = 0;

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;

        while (opMode.opModeIsActive()) {
            //driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            LinearToExpo(-gamepad1.left_stick_y),
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));
            drive.updatePoseEstimate();


            claws.updateRightClawServoRotation();
            claws.updateLeftClawServoRotation();

            if (gamepad2.right_trigger >= 0.4) { //split
                claws.rotateWheels(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger >= 0.4) {
                claws.rotateWheels(-1);
            }
            else if (gamepad2.right_bumper) {
                virtualClawPose = 90;
                lastPIDPower = claws.getPIDArmPower();
                claws.rotateArm(lastPIDPower);
                claws.setArmTargetPosition(virtualClawPose);
            }
            else if (gamepad2.left_bumper) {
                virtualClawPose = 0;
                lastPIDPower = claws.getPIDArmPower();
                claws.rotateArm(lastPIDPower);
                claws.setArmTargetPosition(virtualClawPose);
            }
            else {
                claws.rotateArm(DifferentialClaws.ClawPowerState.OFF.state);
                claws.rotateArm(-gamepad2.left_stick_y); //- Math.cos(Math.toRadians((claws.getActualArmRotation()/claws.maxPoint)*120. - 30.)) * claws.f
            }

            if (Math.abs(gamepad2.right_stick_y) > joystickTolerance) {
                if(horElevatorPosition < 0){
                    horElevatorPosition = 0;
                }else if(horElevatorPosition >= Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state){
                    horElevatorPosition =  Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state;
                }
                //horElevatorPosition += -gamepad2.right_stick_y*40*3;
                //elevators.motorSetHorizontalDestination((int)(horElevatorPosition));
                elevators.motorSetHorizontalPower(-gamepad2.right_stick_y);
            }
            opMode.telemetry.addData("hor motor: ", elevators.horMotor.getPower());
            opMode.telemetry.addData("virtual Pos:", virtualClawPose);
            opMode.telemetry.addData("precieved hor position: ", horElevatorPosition);
            opMode.telemetry.addData("hor position: ", elevators.motorGetHorizontalPosition());
//            telemetry.addData("Control Hub auxillary volts: ", controlHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Expansion Hub auxillary volts: ", expansionHub.getAuxiliaryVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Control Hub used volts: ", controlHub.getInputVoltage(VoltageUnit.VOLTS));
//            telemetry.addData("Expansion Hub used volts: ", expansionHub.getInputVoltage(VoltageUnit.VOLTS));
            opMode.telemetry.update();

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
                elevators.setVerticalDestination(Elevators.VerticalState.VERTICAL_HIGH.state);
            }
            flagElevatorVerticalDpadRight = !gamepad2.dpad_right;

            opMode.telemetry.addData("vert pos:", elevators.getVerticalCurrentPosition());

            // Read line 105
//            AnalogueExtensionVertical = -gamepad2.left_stick_y;
//            elevators.setVerticalDestination((int)(elevators.getVerticalDestination() + AnalogueExtensionVertical * VerticalAnalogueFactor));
//
            if(gamepad2.cross && flagElevatorHorizontalX) {
                elevators.motorSetHorizontalDestination(Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state);
                horElevatorPosition = Elevators.MotorHorizontalState.HORIZONTAL_EXTENDED.state;
            }
            flagElevatorHorizontalX = !gamepad2.cross;
//
            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                elevators.motorSetHorizontalDestination(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED.state);
                horElevatorPosition = Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED.state;
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;
//
//            if(gamepad2.square && flagElevatorHorizontalSquare){
//                elevators.setHorizontalPosition(Elevators.HorizontalState.HORIZONTAL_DROP.state);
//            }
//            flagElevatorHorizontalSquare = !gamepad2.square;
//
//
            if(gamepad2.circle && flagElevatorHorizontalCircle){
                elevators.motorSetHorizontalDestination(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY.state);
                horElevatorPosition = Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY.state;
            }
            flagElevatorHorizontalCircle = !gamepad2.circle;
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


//            Currently Broken, might cause damage to robot
//            AnalogueExtensionHorizontal = -gamepad2.right_stick_x;
//            elevators.setHorizontalPosition(elevators.getHorizontalState() + AnalogueExtensionHorizontal * HorizontalAnalogueFactor);
//
//            telemetry.addData("Epsilon: ", epsilon);
//            telemetry.addData("Right trigger thing: ", gamepad2.right_trigger);
//            telemetry.addData("flag claw take in: ", flagClawTakeIn);
//
//            // take in trigger
//            if(gamepad2.right_trigger > epsilon  && flagClawTakeIn){
//                telemetry.addLine("Should be taking in ");
//                claw.setState(Claws.ClawState.TAKE_IN);
//            }
//            flagClawTakeIn = !(gamepad2.right_trigger > epsilon);
//
//            // spit trigger
//            if(gamepad2.right_bumper && flagClawSpit){
//                claw.setState(Claws.ClawState.SPIT);
//            }
//            flagClawSpit = !gamepad2.right_bumper;
//
//            // claw off
//            if(!gamepad2.right_bumper && !(gamepad2.right_trigger > epsilon)){
//                claw.setState(Claws.ClawState.OFF);
//            }
//
//
//            telemetry.addData("position:", drive.pose);
//            telemetry.addData("right_bumper:", gamepad2.right_trigger);
//            telemetry.addData("right_trigger:", gamepad2.right_bumper);
//            telemetry.update();
        }

    }
    public static double LinearToExpo(double input) {
        if (input >= 0) return input*input;
        else return -input*input;
    }
}
