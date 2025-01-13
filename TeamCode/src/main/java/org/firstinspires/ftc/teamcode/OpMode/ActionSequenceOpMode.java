package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.MecanumDrive.linearInputToExponential;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Disabled
@TeleOp(name = "ActionSequentialOpMode")
public class ActionSequenceOpMode extends LinearOpMode {

    double joystickTolerance = 0.5;
    Elevators elevators;
    DifferentialClaws claws;
    ActionControl control;
    MecanumDrive drive;
    boolean isSampleMode = true; // does the claw interact with samples (take in / spit) or move around

    @Override
    public void runOpMode() throws InterruptedException {
        elevators = new Elevators(this);
        claws = new DifferentialClaws(this);
        control = new ActionControl(elevators, claws);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            /*
            base idea for controls:
                driving is as usual
                systems:
                    claw stuff will be controlled with left_stick_y, toggling using right_trigger
                    picking up is controlled with dpad, and putting in basket stuff with normal buttons
             */
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            linearInputToExponential(gamepad1.left_stick_y),
                            linearInputToExponential(gamepad1.left_stick_x)
                    ),
                    linearInputToExponential(gamepad1.right_stick_x)
            ));
        }
    }
}
