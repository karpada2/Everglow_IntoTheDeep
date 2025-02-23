package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="RightPath", group="Autonomous")
public class RightPath extends LinearOpMode {

    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses
        Pose2d specimins_beginPose = new Pose2d(16, -62,   Math.PI/2);
        Pose2d hangPose = new Pose2d(6,-32,Math.PI/2);;
        double VelConstraint = 20;

        double dropSpeciminY = -52;
        double firstSpeciminX = 46;
        double secondSpeciminX = 60;
        double PickSpeciminY = -12;

        Pose2d specimins_basketPose = new Pose2d(0,-34,Math.PI/2);
        Pose2d specimins_endPose = new Pose2d(23,-10,0);
        Pose2d specimins_pickupPose = new Pose2d(48,-55,-Math.PI/2);
        // Init Systems
        DifferentialClaws claws  = new DifferentialClaws(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, specimins_beginPose);
        Elevators elevators  = new Elevators(this);

        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        ActionControl actionControl = new ActionControl(elevators,claws,colorSensorSystem,drive,gamepad1,gamepad2);
        //Init Trajectories

        TrajectoryActionBuilder B_temp = drive.actionBuilder(specimins_beginPose)
                .strafeTo(specimins_basketPose.position)
                // Lower the vertical elevator
                .waitSeconds(1)
                .setTangent(-Math.PI/4)
//                .strafeToSplineHeading(specimins_dropPose.position, specimins_dropPose.heading)

                .splineToConstantHeading(new Vector2d(33,-33),Math.PI/2)
                .splineToSplineHeading(new Pose2d(firstSpeciminX,PickSpeciminY,Math.PI),0)
//                .setTangent(-Math.PI/2)
//                .splineToConstantHeading(new Vector2d(firstSpeciminX,PickSpeciminY-50),-Math.PI/2)
//                .waitSeconds(0.01)
//                .waitSeconds(0.001)
                .strafeToConstantHeading(new Vector2d(firstSpeciminX,dropSpeciminY),new TranslationalVelConstraint(VelConstraint))
                // Moves the second sample
//                .setTangent(Math.PI * 0.3)
                .splineToConstantHeading(new Vector2d(secondSpeciminX-4,PickSpeciminY),0)
                .waitSeconds(1)
//                .setTangent(-Math.PI/2)
                .strafeTo(new Vector2d(secondSpeciminX,dropSpeciminY),new TranslationalVelConstraint(VelConstraint))
                .setTangent(Math.PI/2)
//                .splineToSplineHeading(new Pose2d(specimins_pickupPose.position.x,specimins_pickupPose.position.y+10, -Math.PI/2),Math.PI)
                .splineToLinearHeading(new Pose2d(specimins_pickupPose.position,-Math.PI/2),-Math.PI/2)

                // Goes to the basket
                .setTangent((1) * Math.PI)
                .strafeToSplineHeading(specimins_basketPose.position, specimins_basketPose.heading)
                //Hangs Specimin

                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                // Hangs Specimin
                .strafeToLinearHeading(specimins_basketPose.position,Math.PI/2)
                // Park
                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI);;

        ;
        TrajectoryActionBuilder B_preload = drive.actionBuilder(specimins_beginPose)
                .strafeTo(hangPose.position);

        TrajectoryActionBuilder B_park = B_preload.endTrajectory().fresh()
                .setTangent(-Math.PI/4)
                .splineToSplineHeading(new Pose2d(60,-60,Math.PI),0);

        Action temp = B_temp.build();

        Action preload = B_preload.build();

        Action park = B_park.build();

        waitForStart();

        Actions.runBlocking(
//                new SequentialAction(
//                        new ParallelAction(
//                                preload,
//                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH)
//                        ),
//                        actionControl.hangSpecimenHigh(),
//                        park
//                )
                temp
        );
    }
}
