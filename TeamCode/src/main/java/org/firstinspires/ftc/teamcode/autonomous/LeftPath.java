package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
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
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        Pose2d beginPose = new Pose2d(-20, -63,   (1./2)*Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Claws claws  = new Claws(this);
        Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder sample1 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-36,-34),0.75*Math.PI);

        TrajectoryActionBuilder unload1 = sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI);

        TrajectoryActionBuilder sample2 = unload1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-48, -34),0.75*Math.PI);

        TrajectoryActionBuilder unload2 = sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI);

        TrajectoryActionBuilder sample3 = unload2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-57, -34),0.75*Math.PI);

        TrajectoryActionBuilder unload3 = sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI);

        TrajectoryActionBuilder sample4 = unload3.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToSplineHeading(new Pose2d(-26,0, 0),0);

        TrajectoryActionBuilder unload4 = sample4.endTrajectory().fresh()
                .setTangent(-(0.75)*Math.PI)
                .splineToSplineHeading(new Pose2d(-56,-56,1.25*Math.PI),1.25*Math.PI);

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        Actions.runBlocking(
                new SequentialAction(
                        sample1,
                        new ParallelAction(
                                //lift elevator
                                unload1,
                        ),
                        //release sample
                        new ParallelAction(
                                //lower elevator
                                sample2,
                        ),
                        //pick up sample
                        new ParallelAction(
                                //lift elevator
                                unload2,
                        ),
                        //release sample

                        new ParallelAction(
                                //lower elevator
                                sample3,
                                ),
                        //pick up sample
                        new ParallelAction(
                                //lift elevator
                                unload3,
                                ),
                        //release sample

                        new ParallelAction(
                                //lower elevator
                                sample4,
                                ),
                        //pick up sample
                        new ParallelAction(
                                //lift elevator
                                unload4,
                                )


                )
        );
    }
}
