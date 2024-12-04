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
import org.firstinspires.ftc.teamcode.Systems.ClawsActionBuilder;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.ElevatorsActionBuilder;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        Pose2d beginPose = new Pose2d(-20, -63,   (1./2)*Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ClawsActionBuilder claws  = new ClawsActionBuilder(this);
        Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder B_sample1 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-28,-37),0.75*Math.PI);

        TrajectoryActionBuilder B_unload1 = B_sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-53,-53),1.25*Math.PI);

        TrajectoryActionBuilder B_sample2 = B_unload1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-45, -37),0.75*Math.PI);

        TrajectoryActionBuilder B_unload2 = B_sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-53,-53),1.25*Math.PI);

        TrajectoryActionBuilder B_sample3 = B_unload2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-54, -37),0.75*Math.PI);

        TrajectoryActionBuilder B_unload3 = B_sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-53,-53),1.25*Math.PI);

        TrajectoryActionBuilder B_sample4 = B_unload3.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToSplineHeading(new Pose2d(-29,0, 0),0);

        TrajectoryActionBuilder B_unload4 = B_sample4.endTrajectory().fresh()
                .setTangent(-(0.75)*Math.PI)
                .splineToSplineHeading(new Pose2d(-53,-53,1.25*Math.PI),1.25*Math.PI);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(3)
                .build();

        // Turning action builders into actions
        Action sample1 = B_sample1.build();
        Action unload1 = B_unload1.build();

        Action sample2 = B_sample2.build();
        Action unload2 = B_unload2.build();

        Action sample3 = B_sample3.build();
        Action unload3 = B_unload3.build();

        Action sample4 = B_sample4.build();
        Action unload4 = B_unload4.build();

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        sample1,
                        unload1,
                        sample2,
                        unload2,
                        sample3,
                        unload3,
                        sample4,
                        unload4)

                );
    }
}
