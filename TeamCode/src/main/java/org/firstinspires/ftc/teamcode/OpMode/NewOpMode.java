package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Claws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Elevators.ElevatorState;


public class NewOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Claws claw = new Claws(this);
        Elevators elevator = new Elevators(this);


        waitForStart();

        double epsilon = 0.4;
        boolean flagElevatorVerticalDpadDown = true;
        boolean flagElevatorVerticalDpadLeft = true;
        boolean flagElevatorVerticalDpadUp = true;
        boolean flagElevatorVerticalDpadRight = true;

        boolean flagElevatorHorizontalX = true;
        boolean flagElevatorHorizontalTriangle = true;
        boolean flagElevatorHorizontalCircle = true;

        boolean flagClawTakeIn = true;
        boolean ClawState = true;
        boolean flagClawSpit = true;

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();


            if(gamepad2.dpad_down){
                elevator.setVerticalDestination((int)ElevatorState.VERTICAL_PICKUP.state);
                flagElevatorVerticalDpadDown = false;
            }
            flagElevatorVerticalDpadDown = !gamepad2.dpad_down;

            if(gamepad2.dpad_left){
                elevator.setVerticalDestination((int)ElevatorState.VERTICAL_HURDLE.state);
                flagElevatorVerticalDpadLeft = false;
            }
            flagElevatorVerticalDpadLeft = !gamepad2.dpad_left;

            if(gamepad2.dpad_up){
                elevator.setVerticalDestination((int)ElevatorState.VERTICAL_LOW.state);
                flagElevatorVerticalDpadUp = false;
            }
            flagElevatorVerticalDpadUp = !gamepad2.dpad_up;

            if(gamepad2.dpad_right){
                elevator.setVerticalDestination((int)ElevatorState.VERTICAL_HIGH.state);
                flagElevatorVerticalDpadRight = false;
            }
            flagElevatorVerticalDpadRight = !gamepad2.dpad_right;


            double VerticalAnalogueFactor = 1;
            double AnalogueExtensionVertical = -gamepad2.left_stick_y;
            elevator.setVerticalDestination((int)(elevator.getVerticalDestination() + AnalogueExtensionVertical * VerticalAnalogueFactor));

            if(gamepad2.x && flagElevatorHorizontalX) {
                elevator.setHorizontalPosition(ElevatorState.HORIZONTAL_EXTENDED.state);
                flagElevatorHorizontalX = false;
            }
            flagElevatorHorizontalX = !gamepad2.x;

            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                elevator.setHorizontalPosition(ElevatorState.HORIZONTAL_RETRACTED.state);
                flagElevatorHorizontalTriangle = false;
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;


            if(gamepad2.circle && flagElevatorHorizontalCircle){
                elevator.setHorizontalPosition(ElevatorState.HORIZONTAL_HALFWAY.state);
                flagElevatorHorizontalCircle = false;
            }
            flagElevatorHorizontalCircle = !gamepad2.circle;

            double HorizontalAnalogueFactor = 1;
            double AnalogueExtensionHorizontal = -gamepad2.right_stick_x;
            elevator.setHorizontalPosition(elevator.getHorizontalState() + AnalogueExtensionHorizontal * HorizontalAnalogueFactor);

            // take in trigger
            if(gamepad2.right_trigger > epsilon  && flagClawTakeIn){
                claw.setState(Claws.ClawState.TAKE_IN);
                flagClawTakeIn = false;
            }
            flagClawTakeIn = !(gamepad2.right_trigger > epsilon);

            // spit trigger
            if(gamepad2.right_bumper && flagClawSpit){
                claw.setState(Claws.ClawState.SPIT);
                flagClawSpit = false;
            }
            flagClawSpit = !gamepad2.right_bumper;

            // claw off
            if(!gamepad2.right_bumper && !(gamepad2.right_trigger > epsilon)){
                claw.setState(Claws.ClawState.OFF);
            }


        }

    }
}