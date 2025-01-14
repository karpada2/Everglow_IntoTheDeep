package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses
        Pose2d beginPose = new Pose2d(-31.1, -63,   Math.PI);
        Pose2d basketPose = new Pose2d(-53,-53,1.25*Math.PI);

        // Init Systems
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DifferentialClaws claws  = new DifferentialClaws(this);
        Elevators elevators  = new Elevators(this);
        //Init Trajectories
        TrajectoryActionBuilder B_preload = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(basketPose.position,basketPose.heading)
                .waitSeconds(1.5);

        TrajectoryActionBuilder B_sample1pickup = B_preload.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-48,-50),0.5*Math.PI);

        TrajectoryActionBuilder B_sample1basket = B_sample1pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample2pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -50),0.5*Math.PI);

        TrajectoryActionBuilder B_sample2basket = B_sample2pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_park = B_sample2basket.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(-24,-10, 0),0);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(1.5)
                .build();

        Action BackAndForth = B_sample1pickup.endTrajectory().fresh()
                .waitSeconds(1)
                .lineToY(-40)
                .waitSeconds(0.2)
                .lineToY(-50)
                .build();

        // Turning action builders into actions
        Action armUp = claws.clawMovementAction(120);

        Action unload = new SequentialAction(claws.clawMovementAction(90),
                                             claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT),
                                             wait,
                                             claws.clawMovementAction(120));
        Action pickup = new ParallelAction(BackAndForth,
                                            new SequentialAction(claws.clawMovementAction(0),
                                                                 claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN)));

        Action preload = B_preload.build();

        Action sample1pickup = B_sample1pickup.build();
        Action sample1basket = B_sample2basket.build();

        Action sample2pickup = B_sample2pickup.build();
        Action sample2basket = B_sample2basket.build();

        Action Park = B_park.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        armUp,
                        wait,
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                        wait,
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY)
                )
        );
/*        Actions.runBlocking(
                new SequentialAction(
                        armUp,//arm up
                        wait,
                        preload, //movement
                        wait,
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),//elevators up
                        wait,
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                        wait,
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT),//unload,
                        wait,
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED),
                        wait,
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)//elevators down,
                        sample1pickup, //movement
                        pickup,
                        armUp,//arm up
                        sample1basket, //movement
                        wait,
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),//elevators up
                        wait,
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                        sample2pickup, //movement
                        pickup,
                        armUp,//arm up
                        sample2basket, //movement
                        wait,
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)//elevators down
                        //Park
                        )
                );*/
    }
}
