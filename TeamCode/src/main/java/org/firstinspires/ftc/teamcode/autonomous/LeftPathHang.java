package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Sweeper;

@Config
@Autonomous(name="LeftPathHang", group="Autonomous")
public class LeftPathHang extends LinearOpMode {
    ActionControl control;
    public static int DownTime = 400;
    public static double collectLine = -50;
    public static double collectLineSampleThree = -40;
    public static double sampleOffset = 3.5;
    public static double addition = 7;
    public static double VelConstraint = 5;

    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses
        Pose2d beginPose = new Pose2d(-31.1, -63,   Math.PI);
        Pose2d basketPose = new Pose2d(-57,-55.3,1.25*Math.PI);
        Pose2d hanging_pose = new Pose2d(-2,-30,0.5*Math.PI);
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1), gamepadEx2 = new GamepadEx(gamepad2);

        // Init Systems
        DifferentialClaws claws  = DifferentialClaws.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Elevators elevators  = Elevators.getInstance(this);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        control = new ActionControl(elevators, claws, colorSensorSystem, drive, new Sweeper(this),
                gamepadEx1, gamepadEx2);
        //Init Trajectories
        TrajectoryActionBuilder B_preload = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(hanging_pose.position,hanging_pose.heading);

        TrajectoryActionBuilder B_sample1pickup = B_preload.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-50 + sampleOffset,collectLine),0.5*Math.PI, new AngularVelConstraint(Math.PI/2));
//                .setTangent(-0.5*Math.PI) //replace the above with this to not hit the sub
//                .splineToConstantHeading(new Vector2d(-50 + sampleOffset,collectLine),-Math.PI);

        TrajectoryActionBuilder B_sample1basket = drive.actionBuilder(new Pose2d(-50 + sampleOffset,collectLine+10,0.5*Math.PI))
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample2pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-60 + sampleOffset, collectLine),0.5*Math.PI);

        TrajectoryActionBuilder B_sample3pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(collectLineSampleThree, -23),Math.PI, new TranslationalVelConstraint(VelConstraint));

        TrajectoryActionBuilder B_sample2basket = drive.actionBuilder(new Pose2d(-60 + sampleOffset,collectLine+10,0.5*Math.PI))
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample3basket = B_sample3pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_park = B_sample3basket.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(-22,-10, 0),0);

        Action BackAndForth1 = drive.actionBuilder(new Pose2d(-50 + sampleOffset,collectLine,0.5*Math.PI))
                .lineToY(collectLine+addition, new TranslationalVelConstraint(VelConstraint))
                .waitSeconds(1)
                .build();

        Action BackAndForth2 = drive.actionBuilder(new Pose2d(-60 + sampleOffset,collectLine,0.5*Math.PI))
                .lineToY(collectLine+addition, new TranslationalVelConstraint(VelConstraint))
                .waitSeconds(0.1)
                .build();

        Action BackAndForth3 = drive.actionBuilder(new Pose2d(collectLineSampleThree, -25,Math.PI))
                //   .waitSeconds(0.1)
                .lineToX(collectLineSampleThree-addition, new TranslationalVelConstraint(VelConstraint))
                .waitSeconds(0.1)
                .lineToX(collectLineSampleThree, new TranslationalVelConstraint(VelConstraint))
                .build();
        // Turning action builders into actions
        //Action armUp = claws.clawMovementAction(DownTime);

        Action unload2 =
                new SequentialAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750)
                );
        Action unload3 = new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750)
        );
        Action unload4 = new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750)
        );
        Action pickup1 = new ParallelAction(BackAndForth1,
                new SequentialAction(
                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                ));
        Action pickup2 = new ParallelAction(BackAndForth2,
                new SequentialAction(
                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                ));

        Action pickup3 = new ParallelAction(BackAndForth3,
                new SequentialAction(
                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),
                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED)
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
                                preload, //movement
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH)
                        ),
                        control.hangSpecimenHigh(),

                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                                sample1pickup //movement
                        ),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                        pickup1,

                        new ParallelAction(
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),
                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
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
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),
                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
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
                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_LOW),
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                                Park  //movement
                        )
                )
        );
    }
}
