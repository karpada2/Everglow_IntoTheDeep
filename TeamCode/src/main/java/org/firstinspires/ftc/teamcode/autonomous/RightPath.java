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
    public class AddToTelemetryAction implements Action {
        private final Telemetry telemetry;
        private final String title;
        private final double value;

        public AddToTelemetryAction(Telemetry telemetry, String title, double value) {
            this.telemetry = telemetry;
            this.title = title;
            this.value = value;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData(title,value);
            telemetry.update();
            return false;
        }
    }
    double dropSpeciminY = -52;
    double firstSpeciminX = 48;
    double secondSpeciminX = 60;
    double PickSpeciminY = -12;
    Pose2d specimins_dropPose = new Pose2d(31.1, -57,   Math.PI/2);
    Pose2d specimins_basketPose = new Pose2d(0,-32,Math.PI/2);
    Pose2d specimins_endPose = new Pose2d(23,-10,0);


    public static double collectLine = -50;
    public static double collectLineSampleThree = -40;
    public static double sampleOffset = 3.5;
    public static double VelConstraint = 10;

    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses
        Pose2d beginPose = new Pose2d(-31.1, -63,   Math.PI);
        Pose2d basketPose = new Pose2d(-55.1,-55.1,1.25*Math.PI);

        // Init Systems
        DifferentialClaws claws  = new DifferentialClaws(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Elevators elevators  = new Elevators(this);

        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        ActionControl actionControl = new ActionControl(elevators,colorSensorSystem,drive);
        //Init Trajectories
        TrajectoryActionBuilder B_preload = drive.actionBuilder(beginPose)
                .strafeTo(specimins_basketPose.position)
                // Lower the vertical elevator

//                .setTangent(-0.3)
//
//                .splineToConstantHeading(new Vector2d(33,-35),Math.PI/6)
//                .splineToSplineHeading(new Pose2d(firstSpeciminX-7,PickSpeciminY,Math.PI/2),0)
//
//                .splineToConstantHeading(new Vector2d(firstSpeciminX,dropSpeciminY),-Math.PI/3)
//
//                .splineToConstantHeading(new Vector2d(secondSpeciminX-3,PickSpeciminY),0)
//                .waitSeconds(1)
//                .setTangent(-Math.PI/2)
//                .strafeTo(new Vector2d(secondSpeciminX,dropSpeciminY))
                ;

        TrajectoryActionBuilder B_park = B_preload.endTrajectory().fresh()
                .setTangent(-Math.PI/4)
                .splineToSplineHeading(new Pose2d(60,-60,Math.PI),0)


        Action unload1 = new SequentialAction(
//                new ParallelAction(
//                elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(270, 750), //down
//                ),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,1000),
//                new ParallelAction(
                claws.clawMovementAction(290, 750)
//                elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED)
//                )
        );

//        Action pickup1 = new ParallelAction(BackAndForth1,
//                new SequentialAction(elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
//                        claws.clawMovementAction(0, 750),
//                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 1000), //colorSensorSystem
//                        claws.clawMovementAction(280, 1500),
//                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED)
//                ));
        Action preload = B_preload.build();

        Action park = B_park.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        preload,

                        park
                )
        );
    }
}
