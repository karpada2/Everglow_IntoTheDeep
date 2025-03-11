package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.Systems.DifferentialClaws.maxPoint;
import static org.firstinspires.ftc.teamcode.tuning.ClawPIDFTuning.f;

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
    private final MecanumDrive drive;

    public DriversOpMode(LinearOpMode opMode, Gamepad gamepad1, Gamepad gamepad2){
        this.opMode = opMode;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));
    }

    public void run(boolean isBlue){

        DifferentialClaws claws = DifferentialClaws.getInstance(opMode);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(opMode, isBlue);
        Elevators elevators = Elevators.getInstance(opMode);
        elevators.setVerticalPower(0.0);
        boolean isInitialized = false;
        boolean secondery = false;
        //ActionControl control = new ActionControl(elevators, claws, colorSensorSystem, drive, gamepad1, gamepad2);
        //Thread driverThread = new Thread(this::DriverRun);
        //driverThread.start();
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

        double ff;
        boolean leftBumper = true;
        boolean rightBumper = true;
        boolean isPIDFRuns = false;
        double lastPIDPower = 0;
        double virtualClawPose = maxPoint;

        double horElevatorPosition = 0;
        double startTime = 0;

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;
        boolean isRunPID = false;
        int loopsDone = 0;
        double timeOfStart = System.currentTimeMillis();
        double timeSinceStartSecs = (System.currentTimeMillis() - timeOfStart)/1000.0;
        
        while (opMode.opModeIsActive()) {
            loopsDone++;
            timeSinceStartSecs = (System.currentTimeMillis() - timeOfStart)/1000.0;
            //driving
            ff = Math.cos(Math.toRadians((claws.getActualArmRotation()/maxPoint)*120. - 30.)) * f;
            claws.updateRightClawServoRotation();
            claws.updateLeftClawServoRotation();

            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                virtualClawPose = DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state;
                claws.setArmTargetPosition(virtualClawPose);
                startTime = System.currentTimeMillis();
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;

            if (gamepad2.right_trigger >= 0.4) { //split
                claws.rotateWheels(DifferentialClaws.ClawPowerState.TAKE_IN);
            }
            else if (gamepad2.left_trigger >= 0.4) {
                claws.rotateWheels(-gamepad2.left_trigger/2);
            }
            else if (gamepad2.right_bumper && rightBumper) {
                virtualClawPose = DifferentialClaws.ClawPositionState.SPIT_STATE.state;
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
                if(!isPIDFRuns)
                    claws.rotateArm(-(ff + gamepad2.left_stick_y/100)); //- Math.cos(Math.toRadians((claws.getActualArmRotation()/claws.maxPoint)*120. - 30.)) * claws.f
            }

            isPIDFRuns = System.currentTimeMillis()-startTime < 1500;

            leftBumper = !gamepad2.left_bumper;
            rightBumper = !gamepad2.right_bumper;

            if (Math.abs(gamepad2.right_stick_y) > joystickTolerance) {
                if(horElevatorPosition < 0){
                    horElevatorPosition = 0;
                }else if(horElevatorPosition >= Elevators.HorizontalState.HORIZONTAL_EXTENDED.state){
                    horElevatorPosition =  Elevators.HorizontalState.HORIZONTAL_EXTENDED.state;
                }
                horElevatorPosition += 1.5*4*(-gamepad2.right_stick_y/100.);
                elevators.setHorizontalDestination(horElevatorPosition);
            }
            opMode.telemetry.addData("sweeper:", virtualClawPose);
            opMode.telemetry.addData("precieved hor position: ", horElevatorPosition);
            opMode.telemetry.addData("hor position: ", elevators.getHorizontalDestination());
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

            if(isPIDFRuns) {
                lastPIDPower = claws.getPIDArmPower();
                claws.rotateArm(lastPIDPower);
//                if ((claws.getActualArmRotation() <= 5 && claws.getArmTargetPosition() ==0)
//                        || (claws.getActualArmRotation() >= maxPoint-2 && claws.getArmTargetPosition() == maxPoint))
//                    lastPIDPower = 0;
            }
            opMode.telemetry.addData("loops done", loopsDone);
            opMode.telemetry.addData("time since start", timeSinceStartSecs);
            opMode.telemetry.addData("loops per second avg", loopsDone/timeSinceStartSecs);
            opMode.telemetry.update();
        }

    }

    private void DriverRun(){
        Sweeper sweeper = new Sweeper(opMode);

        opMode.waitForStart();
        while (opMode.opModeIsActive()){
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
        }
    }
    public static double LinearToExpo(double input) {
        if (input >= 0) return input*input;
        else return -input*input;
    }
}
