package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Claws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "FirstOpMode")
public class NewOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        Claws claw = new Claws(this);
        Elevators elevator = new Elevators(this);
        elevator.setVerticalPower(0.8);

        waitForStart();

        double epsilon = 0.4;
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

        double AnalogueExtensionVertical;
        double VerticalAnalogueFactor = 1;

        double HorizontalAnalogueFactor = 1;
        double AnalogueExtensionHorizontal;

        while (opModeIsActive()) {
            //driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x
                    ),
                    gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();


            if(gamepad2.dpad_down && flagElevatorVerticalDpadDown) {
                elevator.setVerticalDestination(Elevators.VerticalState.VERTICAL_PICKUP.state);
            }
            flagElevatorVerticalDpadDown = !gamepad2.dpad_down;

            if(gamepad2.dpad_left && flagElevatorVerticalDpadLeft){
                elevator.setVerticalDestination(Elevators.VerticalState.VERTICAL_HURDLE.state);
            }
            flagElevatorVerticalDpadLeft = !gamepad2.dpad_left;

            if(gamepad2.dpad_up && flagElevatorVerticalDpadUp){
                elevator.setVerticalDestination(Elevators.VerticalState.VERTICAL_LOW.state);
            }
            flagElevatorVerticalDpadUp = !gamepad2.dpad_up;

            if(gamepad2.dpad_right && flagElevatorVerticalDpadRight){
                elevator.setVerticalDestination(Elevators.VerticalState.VERTICAL_HIGH.state);
            }
            flagElevatorVerticalDpadRight = !gamepad2.dpad_right;


            // Read line 105
//            AnalogueExtensionVertical = -gamepad2.left_stick_y;
//            elevator.setVerticalDestination((int)(elevator.getVerticalDestination() + AnalogueExtensionVertical * VerticalAnalogueFactor));

            if(gamepad2.cross && flagElevatorHorizontalX) {
                elevator.setHorizontalPosition(Elevators.HorizontalState.HORIZONTAL_EXTENDED.state);
            }
            flagElevatorHorizontalX = !gamepad2.cross;

            if(gamepad2.triangle && flagElevatorHorizontalTriangle){
                elevator.setHorizontalPosition(Elevators.HorizontalState.HORIZONTAL_RETRACTED.state);
            }
            flagElevatorHorizontalTriangle = !gamepad2.triangle;

//            if(gamepad2.square && flagElevatorHorizontalSquare){
//                elevator.setHorizontalPosition(Elevators.HorizontalState.HORIZONTAL_DROP.state);
//            }
//            flagElevatorHorizontalSquare = !gamepad2.square;


            if(gamepad2.circle && flagElevatorHorizontalCircle){
                elevator.setHorizontalPosition(Elevators.HorizontalState.HORIZONTAL_HALFWAY.state);
            }
            flagElevatorHorizontalCircle = !gamepad2.circle;

            //Currently Broken, might cause damage to robot
//            AnalogueExtensionHorizontal = -gamepad2.right_stick_x;
//            elevator.setHorizontalPosition(elevator.getHorizontalState() + AnalogueExtensionHorizontal * HorizontalAnalogueFactor);

            telemetry.addData("Epsilon: ", epsilon);
            telemetry.addData("Right trigger thing: ", gamepad2.right_trigger);
            telemetry.addData("flag claw take in: ", flagClawTakeIn);

            // take in trigger
            if(gamepad2.right_trigger > epsilon  && flagClawTakeIn){
                telemetry.addLine("Should be taking in ");
                claw.setState(Claws.ClawState.TAKE_IN);
            }
            flagClawTakeIn = !(gamepad2.right_trigger > epsilon);

            // spit trigger
            if(gamepad2.right_bumper && flagClawSpit){
                claw.setState(Claws.ClawState.SPIT);
            }
            flagClawSpit = !gamepad2.right_bumper;

            // claw off
            if(!gamepad2.right_bumper && !(gamepad2.right_trigger > epsilon)){
                claw.setState(Claws.ClawState.OFF);
            }


            telemetry.addData("position:", drive.pose);
            telemetry.addData("right_bumper:", gamepad2.right_trigger);
            telemetry.addData("right_trigger:", gamepad2.right_bumper);
            telemetry.update();
        }

    }
}