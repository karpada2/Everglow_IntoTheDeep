package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.linearInputToExponential;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Claws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;


@TeleOp(name="Sequence OpMode")
public class SequenceOpMode extends LinearOpMode {

    double trigger_threshold = 0.5;

    Elevators elevators;
    Claws claw;
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

    @Override
    public void runOpMode() throws InterruptedException {
//        Actions.runBlocking(
//                new SequentialAction(
//                        elevators.vertMoveTo(Elevators.VerticalState.VERTICAL_MIN),
//                        elevators.horMoveTo(Elevators.HorizontalState.HORIZONTAL_RETRACTED)
//                )
//        );

        claw = new Claws(this);
        elevators = new Elevators(this);
        SequenceControl sequenceControl = new SequenceControl(elevators, claw);
        SequenceRunner sequenceRunner = new SequenceRunner();

        boolean flagRightBumper = true;
        boolean flagLeftBumper = true;

        boolean flagTriangle = true;
        boolean flagCircle = true;
        boolean flagSquare = true;

        boolean flagDpadUp = true;
        boolean flagDpadLeft = true;

        waitForStart();

        Claws.ClawState currentClawState = Claws.ClawState.OFF;

        while (opModeIsActive()) {
            // driving
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            linearInputToExponential(-gamepad1.left_stick_y),
                            linearInputToExponential(-gamepad1.left_stick_x)
                    ),
                    linearInputToExponential(-gamepad1.right_stick_x)
            ));

            drive.updatePoseEstimate();



            if (gamepad2.right_bumper && flagRightBumper) {
                if (currentClawState == Claws.ClawState.OFF || currentClawState == Claws.ClawState.SPIT) {
                    currentClawState = Claws.ClawState.TAKE_IN;
                }
                else {
                    currentClawState = Claws.ClawState.OFF;
                }
            }
            else if (gamepad2.left_bumper && flagLeftBumper) {
                if (currentClawState == Claws.ClawState.OFF || currentClawState == Claws.ClawState.TAKE_IN) {
                    currentClawState = Claws.ClawState.SPIT;
                }
                else {
                    currentClawState = Claws.ClawState.OFF;
                }
            }

            flagRightBumper = !gamepad2.right_bumper;
            flagLeftBumper = !gamepad2.left_bumper;

            Action clawAction = claw.setClawAction(currentClawState);

            Action hurdleAction = null;

            if (gamepad2.triangle && flagTriangle) {
                //pickUp - to halfWay
                sequenceRunner.RunSequence(sequenceControl.halfPickUpSeq);
            }
            else if (gamepad2.square && flagSquare){
                // pickup - to extanded
                sequenceRunner.RunSequence(sequenceControl.extendedPickUpSeq);
            }else if (gamepad2.circle && flagCircle) {
                //return from pickup
                sequenceRunner.RunSequence(sequenceControl.returnFromPickUp);
            }

            flagTriangle = !gamepad2.triangle;
            flagCircle = !gamepad2.circle;
            flagSquare = !gamepad2.square;

            Action basketAction = null;

            if (gamepad2.dpad_left && flagDpadLeft) {
                // low basket
                sequenceRunner.RunSequence(sequenceControl.getReadyToDropLowSeq);
            }
            else if (gamepad2.dpad_up && flagDpadUp) {
                // high basket
                sequenceRunner.RunSequence(sequenceControl.getReadyToDropHighSeq);
            }

            flagDpadLeft = !gamepad2.dpad_left;
            flagDpadUp = !gamepad2.dpad_up;

            sequenceRunner.Update();

            /*
            right bumper - claw takeIn
            left bumper - claw spit
            triangle - action to get ready for pickup (clear hurdle and that)
            circle - action to clear hurdle in the opposite way
            dpad up - action to move to low basket and put the sample in
            dpad left - action to move to high basket and put the sample in
             */
        }
        sequenceRunner.Interapt();
        sleep(500);
    }
}

