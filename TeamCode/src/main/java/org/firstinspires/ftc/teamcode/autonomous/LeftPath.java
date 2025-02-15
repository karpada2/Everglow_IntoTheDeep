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
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {

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
        //Init Trajectories
        TrajectoryActionBuilder B_preload = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample1pickup = B_preload.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-50 + sampleOffset,collectLine),0.5*Math.PI);

        TrajectoryActionBuilder B_sample1basket = B_sample1pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample2pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-60 + sampleOffset, collectLine),0.5*Math.PI);

        TrajectoryActionBuilder B_sample3pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(collectLineSampleThree, -25),Math.PI);

        TrajectoryActionBuilder B_sample2basket = B_sample2pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample3basket = B_sample3pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_park = B_sample3basket.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(-24,-10, 0),0);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .build();

        Action BackAndForth1 = drive.actionBuilder(new Pose2d(-50 + sampleOffset,collectLine,0.5*Math.PI))
              //  .waitSeconds(0.1)
                .lineToY(collectLine+10, new TranslationalVelConstraint(VelConstraint))
                .waitSeconds(1)
                .lineToY(collectLine, new TranslationalVelConstraint(VelConstraint))
                .build();

        Action BackAndForth2 = drive.actionBuilder(new Pose2d(-60 + sampleOffset,collectLine,0.5*Math.PI))
             //   .waitSeconds(0.1)
                .lineToY(collectLine+10, new TranslationalVelConstraint(VelConstraint))
                .waitSeconds(0.1)
                .lineToY(collectLine, new TranslationalVelConstraint(VelConstraint))
                .build();

        Action BackAndForth3 = drive.actionBuilder(new Pose2d(collectLineSampleThree, -25,Math.PI))
             //   .waitSeconds(0.1)
                .lineToX(collectLineSampleThree-10, new TranslationalVelConstraint(VelConstraint))
                .waitSeconds(0.1)
                .lineToX(collectLineSampleThree, new TranslationalVelConstraint(VelConstraint))
                .build();
        // Turning action builders into actions
        //Action armUp = claws.clawMovementAction(100);

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
        Action unload2 =
                new SequentialAction(
//                new ParallelAction(
//                elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(270, 750), //down 135
//                ),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,1000),
//                new ParallelAction(
                        claws.clawMovementAction(290, 750) //260
//                elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED)
//                )
                );
        Action unload3 = new SequentialAction(
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

        Action unload4 = new SequentialAction(
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
        Action pickup1 = new ParallelAction(BackAndForth1,
                new SequentialAction(elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(0, 750),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 1000), //colorSensorSystem
                        claws.clawMovementAction(280, 1500),
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED)
                ));
        Action pickup2 = new ParallelAction(BackAndForth2,
                new SequentialAction(elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(0, 750),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 1000), //colorSensorSystem
                        claws.clawMovementAction(280, 1500),
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED)
                ));

        Action pickup3 = new ParallelAction(BackAndForth3,
                new SequentialAction(elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(0, 750),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 1000), //colorSensorSystem
                        claws.clawMovementAction(250, 1500),
                        elevators.setMotorHorizontalElevatorAction(Elevators.MotorHorizontalState.HORIZONTAL_RETRACTED)
                ));

        Action preload = B_preload.build();

        Action sample1pickup = B_sample1pickup.build();
        Action sample1basket = B_sample1basket.build();

        Action sample2pickup = B_sample2pickup.build();
        Action sample2basket = B_sample2basket.build();

        Action sample3pickup = B_sample3pickup.build();
        Action sample3basket = B_sample3basket.build();

        Action Park = B_park.build();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                preload //movement
                        ),
                        unload1,
                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                                sample1pickup //movement
                        ),
                        pickup1,

                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                sample1basket //movement
                        ),
                        unload2,

                       new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                                sample2pickup //movement
                        ),
                        pickup2,
                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                sample2basket  //movement
                        ),
                        unload3,


                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                                sample3pickup //movement
                        ),

                        pickup3,

                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH),
                                sample3basket  //movement
                        ),
                        unload4,
                        Park
                        )
                );
    }
}
