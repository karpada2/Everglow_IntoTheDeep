package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Systems.Elevators.HorizontalState.HORIZONTAL_EXTENDED;
import static org.firstinspires.ftc.teamcode.Systems.Elevators.HorizontalState.HORIZONTAL_HALFWAY;
import static org.firstinspires.ftc.teamcode.Systems.Elevators.HorizontalState.HORIZONTAL_QUARTERWAY;
import static org.firstinspires.ftc.teamcode.Systems.Elevators.HorizontalState.HORIZONTAL_RETRACTED;

import android.text.style.UpdateAppearance;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
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
import org.firstinspires.ftc.teamcode.Systems.Sweeper;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenParallelAction;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenSequentialAction;

@Config
@Autonomous(name="LeftPath", group="Autonomous")
public class LeftPath extends LinearOpMode {

    public class WaitAction implements Action {
        private double startTime = -1;
        private final double timeToSleep;
        public WaitAction(double timeToSleep) {
            this.timeToSleep = timeToSleep;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (startTime == -1) {
                startTime = System.currentTimeMillis();
            }

            return System.currentTimeMillis() - startTime <= timeToSleep;
        }
    }

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

    //TODO: change these variables so the Autonomous will be optimal
    //----------------------------------------------//
    public static int DownTime = 500, BetweenUpTime = 680, ToUpTime = 1000;
    //----------------------------------------------//
    public static double collectLine = -40;
    public static double collectLineSampleThree = -52;
    public static double firstSampleX = -35;
    public static double sampleOffset1 = 2.5;
    public static double sampleOffset2 = -8;
    public static double VelConstraint = 5;

    @Override
    public void runOpMode() throws InterruptedException{
        // Init Poses
        Pose2d beginPose = new Pose2d(-31.1, -63,   Math.PI);
        Pose2d basketPose = new Pose2d(-54,-53.3,1.25*Math.PI);

        // Init Systems
        DifferentialClaws claws  = DifferentialClaws.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Elevators elevators  = Elevators.getInstance(this);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        Sweeper sweeper = new Sweeper(this);
        //Init Trajectories
        TrajectoryActionBuilder B_preload = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(basketPose.position, basketPose.heading, new AngularVelConstraint(Math.PI*1.5));

        TrajectoryActionBuilder B_sample1pickup = B_preload.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(firstSampleX + sampleOffset1,collectLine),0.75*Math.PI);

        TrajectoryActionBuilder B_sample1basket = B_sample1pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample2pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(firstSampleX + sampleOffset2, collectLine),0.75*Math.PI);

        TrajectoryActionBuilder B_sample3pickup = B_sample1basket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(collectLineSampleThree, -42),Math.PI*0.75);

        TrajectoryActionBuilder B_sample2basket = B_sample2pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_sample3basket = B_sample3pickup.endTrajectory().fresh()
                .strafeToSplineHeading(basketPose.position,basketPose.heading);

        TrajectoryActionBuilder B_park = B_sample3basket.endTrajectory().fresh()
                .setTangent(Math.PI * 0.5)
                .splineToLinearHeading(new Pose2d(-22,-10, 0.25*Math.PI),0, new TranslationalVelConstraint(100));

        TrajectoryActionBuilder B_FinalSample = B_park.endTrajectory().fresh()
                .setTangent(Math.PI)
                .splineToConstantHeading(new Pose2d(-35,-10, 0).position, Math.PI, new TranslationalVelConstraint(100))
                .setTangent(Math.PI)
                .splineToSplineHeading(basketPose,Math.PI*1.25, new TranslationalVelConstraint(60));

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .build();


        // Turning action builders into actions
        //Action armUp = claws.clawMovementAction(DownTime);

        Action preload = B_preload.build();

        Action sample1pickup = B_sample1pickup.build();
        Action sample1basket = B_sample1basket.build();

        Action sample2pickup = B_sample2pickup.build();
        Action sample2basket = B_sample2basket.build();

        Action sample3pickup = B_sample3pickup.build();
        Action sample3basket = B_sample3basket.build();

        Action Park = B_park.build();
        Action dropFive = B_FinalSample.build();

        Action unload1 = new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, BetweenUpTime),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, BetweenUpTime)
        );
        Action unload2 = new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, BetweenUpTime),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, BetweenUpTime)
        );
        Action unload3 = new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, BetweenUpTime),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, BetweenUpTime)
        );
        Action unload4 = new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, BetweenUpTime),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, BetweenUpTime)
        );
        Action pickup1 = new SequentialAction(
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_PICKUP),
                        sample1pickup //movement
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                new ParallelAction(
                        elevators.setHorizontalElevatorAction(HORIZONTAL_HALFWAY),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                )
//                new ParallelAction(
//                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, ToUpTime),
//                        elevators.setHorizontalElevatorAction(HORIZONTAL_RETRACTED)
//                )
        );
        Action pickup2 =  new SequentialAction(
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_PICKUP),
                        sample2pickup //movement
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                new ParallelAction(
                        elevators.setHorizontalElevatorAction(HORIZONTAL_HALFWAY),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                )
//                new ParallelAction(
//                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.READY_TO_SPIT.state, ToUpTime),
//                        elevators.setHorizontalElevatorAction(HORIZONTAL_RETRACTED)
//                )
        );

        Action pickup3 = new SequentialAction(
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_PICKUP),
                        sample3pickup //movement
                ),

                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                elevators.setHorizontalElevatorAction(HORIZONTAL_HALFWAY),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
        );

        Action pickup4 = new SequentialAction(
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_PICKUP)
                        // movement
                ),

                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
//                elevators.setHorizontalElevatorAction(HORIZONTAL_HALFWAY),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
        );

        waitForStart();

        telemetry.addData("claws", claws);
        telemetry.addData("elevators", elevators);
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                preload //movement
                        ),
                        unload1,
                        pickup1,


                        new ParallelAction(
                                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.READY_TO_SPIT.state, ToUpTime),
                                elevators.setHorizontalElevatorAction(HORIZONTAL_RETRACTED),
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1250),
                                //TODO: I don't know if we need the claws Action here. if there is no problem do not touch it.
                                sample1basket //movement
                                ),
                        unload2,
                        pickup2,

                        new ParallelAction(
                                elevators.setHorizontalElevatorAction(HORIZONTAL_RETRACTED),
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1250),
                                sample2basket //movement
                        ),

                        unload3,
                        pickup3,

                        new ParallelAction(
                                elevators.setHorizontalElevatorAction(HORIZONTAL_RETRACTED),
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1250),
                                sample3basket //movement
                        ),

                        unload4,

                        new ParallelAction(
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1200),
                                Park  //movement
                        ),

                        new SequentialAction(
                            sweeper.getSweeperAction(true, 500),
                            claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, DownTime),
                            new ParallelAction(
                                    claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem),
                                    elevators.setHorizontalElevatorAction(HORIZONTAL_HALFWAY)
                            )
                        ),

                        new ParallelAction(
                                new SequentialAction(
                                        new ParallelAction(
                                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
                                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),
                                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH)
                                                ),
                                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750),
                                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem),
                                        new ParallelAction(
                                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),
                                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
                                        )
                                ),
                                dropFive //movement
                        )
                )
        );
    }
}
