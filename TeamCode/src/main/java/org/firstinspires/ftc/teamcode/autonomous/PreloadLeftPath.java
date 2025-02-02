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
@Autonomous(name="PreloadLeftPath", group="Autonomous")
public class PreloadLeftPath extends LinearOpMode {
    @Override
    public void runOpMode()  throws InterruptedException{
        Pose2d beginPose = new Pose2d(-31.1, -63,   Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        DifferentialClaws claws  = new DifferentialClaws(this);
        Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder B_unload0 = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(-57,-57),1.25*Math.PI);

        TrajectoryActionBuilder B_Park = B_unload0.endTrajectory().fresh()
                .setTangent(Math.PI * 0.25)
                .splineToSplineHeading(new Pose2d(-35,-23, -Math.PI * 0.5),Math.PI/2).waitSeconds(1)
                .setTangent(Math.PI/2)
                .splineToSplineHeading(new Pose2d(-25,-10, Math.PI),0);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(1 )
                .build();

        // Turning action builders into actions
        Action unload0 = B_unload0.build();

        Action Park = B_Park.build();

//        Actions.runBlocking(v_e to 0 and h_e to 0)
//        Actions.runBlocking(set elevator power to 0.8)

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        claws.test(3000, 4000),
                new SequentialAction(
                        new ParallelAction(
                                unload0,
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH)// elevators up
                        ),
                        wait,
                        new ParallelAction(
                                Park,
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)// elevators down
                        )))
                );
    }
}
