package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(-20, -63,   (1./2)*Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Claws claws  = new Claws(this);
        Elevators elevators  = new Elevators(this);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-36,-34),0.75*Math.PI);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI)
                .strafeToSplineHeading(new Vector2d(-48, -34),0.75*Math.PI);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-56,-56),1.25*Math.PI)
                .strafeToSplineHeading(new Vector2d(-57, -34),0.75*Math.PI);

    }
}
