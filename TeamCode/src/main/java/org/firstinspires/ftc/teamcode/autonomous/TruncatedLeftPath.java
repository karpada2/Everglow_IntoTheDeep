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

@Config
@Autonomous(name="TruncatedLeftPath", group="Autonomous")
public class TruncatedLeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        Pose2d beginPose = new Pose2d(-31.1, -61,   (1./2)*Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //Claws claws  = new Claws(this);
        //Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder B_unload0 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-51,-51),1.25*Math.PI);

        TrajectoryActionBuilder B_Park = B_unload0.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-31.1, -61),Math.PI/2);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(3)
                .build();

        // Turning action builders into actions
        Action unload0 = B_unload0.build();

        Action Park = B_Park.build();

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        unload0,
                        Park)
                );
    }
}
