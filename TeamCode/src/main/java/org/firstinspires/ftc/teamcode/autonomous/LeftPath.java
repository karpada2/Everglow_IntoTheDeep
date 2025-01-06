package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Claws;
import org.firstinspires.ftc.teamcode.Systems.Claws.ClawState;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators.HorizontalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses
        Pose2d beginPose = new Pose2d(-31.1, -63,   Math.PI);
        Pose2d basket_pose = new Pose2d(-57,-57,1.25*Math.PI);

        // Init Systems
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DifferentialClaws claws  = new DifferentialClaws(this);
        Elevators elevators  = new Elevators(this);

        //Init Trajectories
        TrajectoryActionBuilder B_preload = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(basket_pose.position,basket_pose.heading);

        TrajectoryActionBuilder B_sample1pickup = B_preload.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-48,-40),0.5*Math.PI);

        TrajectoryActionBuilder B_sample1unload = B_sample1pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basket_pose.position,basket_pose.heading);

        TrajectoryActionBuilder B_sample2pickup = B_sample1unload.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -40),0.5*Math.PI);

        TrajectoryActionBuilder B_sample2unload = B_sample2pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basket_pose.position,basket_pose.heading);

        TrajectoryActionBuilder B_park = B_sample2unload.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(-24,-10, 0),0);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(3)
                .build();

        // Turning action builders into actions
        Action preload = B_preload.build();

        Action sample1pickup = B_sample1pickup.build();
        Action sample1unload = B_sample1unload.build();

        Action sample2pickup = B_sample2pickup.build();
        Action sample2unload = B_sample2unload.build();

        Action Park = B_park.build();

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        sample1pickup,
                        sample1unload,
                        sample2pickup,
                        sample2unload,
                        Park)
                );
    }
}
