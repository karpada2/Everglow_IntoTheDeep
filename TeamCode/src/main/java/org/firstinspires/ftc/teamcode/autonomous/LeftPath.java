package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        Pose2d beginPose = new Pose2d(-31.1, -61,   (1./2)*Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //Claws claws  = new Claws(this);
        //Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder B_unload0 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI);

        TrajectoryActionBuilder B_sample1 = B_unload0.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-48,-38),0.5*Math.PI)
                .waitSeconds(1);

        TrajectoryActionBuilder B_unload1 = B_sample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI);

        TrajectoryActionBuilder B_sample2 = B_unload1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-58, -38),0.5*Math.PI)
                .waitSeconds(1);

        TrajectoryActionBuilder B_unload2 = B_sample2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI);

        TrajectoryActionBuilder B_sample3 = B_unload2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-57, -26),Math.PI)
                .waitSeconds(1);

        TrajectoryActionBuilder B_unload3 = B_sample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI);

        TrajectoryActionBuilder B_sample4 = B_unload3.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToSplineHeading(new Pose2d(-34,0, 0),0)
                .waitSeconds(1);;

        TrajectoryActionBuilder B_unload4 = B_sample4.endTrajectory().fresh()
                .setTangent(-(0.75)*Math.PI)
                .splineToSplineHeading(new Pose2d(-51,-51,1.25*Math.PI),1.25*Math.PI);

        TrajectoryActionBuilder B_Park = B_unload4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-31.1, -61),Math.PI/2);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(3)
                .build();

        // Turning action builders into actions
        Action unload0 = B_unload0.build();
        Action sample1 = B_sample1.build();
        Action unload1 = B_unload1.build();

        Action sample2 = B_sample2.build();
        Action unload2 = B_unload2.build();

        Action sample3 = B_sample3.build();
        Action unload3 = B_unload3.build();

        Action sample4 = B_sample4.build();
        Action unload4 = B_unload4.build();

        Action Park = B_Park.build();

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        unload0,
                        unload1,
                        sample2,
                        unload2,
                        sample3,
                        unload3,
                        sample4,
                        unload4,
                        Park)
                );
    }
}
