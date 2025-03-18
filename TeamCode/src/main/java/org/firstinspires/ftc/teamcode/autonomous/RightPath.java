package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;
import androidx.appcompat.app.ActionBar;

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
@Autonomous(name="RightPath", group="Autonomous")
public class RightPath extends LinearOpMode {
    public static int waitSweeperTime = 1200;
    public static int timeToCloseSweeper = 1000;
    public static int spitPos = -35;

    public static int downTime = 500;

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

    @Override
    public void runOpMode()  throws InterruptedException{


        // Init Poses
        Pose2d specimins_beginPose = new Pose2d(16, -62,   Math.PI/2);
        Pose2d hangPose = new Pose2d(6,-32,Math.PI/2);

        double VelConstraint = 20;

        double dropSpeciminY = -52;
        double firstSpeciminX = 45;
        double secondSpeciminX = 48;
        double thirdSpeciminX = 60;
        double PickSpeciminY = -12;

        Pose2d specimins_basketPose = new Pose2d(6,spitPos,Math.PI/2);
        Pose2d specimins_basketPose2 = new Pose2d(2,spitPos,Math.PI/2);
        Pose2d specimins_basketPose3 = new Pose2d(-2,spitPos,Math.PI/2);
        Pose2d specimins_basketPose4 = new Pose2d(-6,spitPos,Math.PI/2);
        Pose2d specimins_pickupPose = new Pose2d(32, -58, Math.PI*1.75);//new Pose2d(48,-52,-Math.PI/2);



        // Init Systems
        DifferentialClaws claws  = DifferentialClaws.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, specimins_beginPose);
        Elevators elevators  = Elevators.getInstance(this);
        Sweeper sweeper = new Sweeper(this);

        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, false);
        ActionControl actionControl = new ActionControl(elevators,claws,colorSensorSystem,drive, new Sweeper(this),new GamepadEx(gamepad1), new GamepadEx(gamepad2));
        //Init Trajectories


        TrajectoryActionBuilder B_preload = drive.actionBuilder(specimins_beginPose)
                //starts at hanging height
                .strafeTo(specimins_basketPose.position)
                //* Hangs
                ;

        TrajectoryActionBuilder B_intake = B_preload.endTrajectory().fresh()
                .setTangent(Math.PI* 1.75)
                .splineToLinearHeading(new Pose2d(40,-36,Math.PI/4),0);

        TrajectoryActionBuilder B_turn = B_intake.endTrajectory().fresh()
                .setTangent(-Math.PI* 0.5)
                .splineToLinearHeading(new Pose2d(40,-50,-Math.PI/2),-Math.PI/2);

        TrajectoryActionBuilder B_intake2 = B_turn.endTrajectory().fresh()
                .setTangent(0.5*Math.PI)
                .splineToSplineHeading(new Pose2d(40, -45, 0.25*Math.PI), Math.PI*0.5)
                .splineToLinearHeading(new Pose2d(50, -36, Math.PI/4),0)
                .splineToLinearHeading(new Pose2d(50,-50,-Math.PI/2),Math.PI/2);

        TrajectoryActionBuilder B_pickup1 = B_intake2.endTrajectory().fresh()
                .setTangent(Math.PI*0.75)
                .splineToLinearHeading(new Pose2d(42, -48, -Math.PI/2),Math.PI*1.75);


        TrajectoryActionBuilder B_hang1 = B_pickup1.endTrajectory().fresh()
                .setTangent(Math.PI*0.75)
                .splineToSplineHeading(specimins_basketPose2,Math.PI*0.75);;

        TrajectoryActionBuilder B_pickup2 = B_hang1.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(42, -48, -Math.PI/2),Math.PI*1.75)
                ;

        TrajectoryActionBuilder B_hang2 = B_pickup2.endTrajectory().fresh()
                .setTangent(Math.PI*0.75)
                .splineToSplineHeading(specimins_basketPose3,Math.PI*0.75)
                ;

        TrajectoryActionBuilder B_pickup3 = B_hang2.endTrajectory().fresh()
                .setTangent(-Math.PI/2)
                .splineToLinearHeading(new Pose2d(42, -48, -Math.PI/2),Math.PI*1.75)
                ;

        TrajectoryActionBuilder B_hang3 = B_pickup2.endTrajectory().fresh()
                .setTangent(Math.PI*0.75)
                .splineToSplineHeading(specimins_basketPose4,Math.PI*0.75);

        TrajectoryActionBuilder B_pickup4 = B_hang3.endTrajectory().fresh()
                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                ;

        TrajectoryActionBuilder B_park = B_pickup2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI)
                ;



        Action m_preload = B_preload.build();

        Action m_intake = B_intake.build();
        Action m_intake2 = B_intake2.build();
        Action m_turn = B_turn.build();
        Action m_pickup = B_pickup1.build();
        Action m_hang1 = B_hang1.build();
        Action m_hang2 = B_hang2.build();
        Action m_hang3 = B_hang3.build();
        Action m_pickup3 = B_pickup3.build();
        Action m_pickup4 = B_pickup4.build();

        Action m_pickup2 = B_pickup2.build();

        Action m_park = B_park.build();

        Action preload = new SequentialAction(
                new ParallelAction(
                        m_preload, //movement
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD)
                ),
                new SequentialAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
                )
        );

        Action intake = new SequentialAction(
                new ParallelAction(
                        m_intake,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        new SequentialAction(
                                new WaitAction(waitSweeperTime),
                                sweeper.getSweeperAction(Sweeper.SweeperAngle.HALF_EXTENDED,0)
                        )
                )
        );

        Action hang1 = new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                            elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED, timeToCloseSweeper),
                             sweeper.getSweeperAction(Sweeper.SweeperAngle.SWEEPER_RETRACTED, 0)
                        ),
                        m_hang1,
                        new SequentialAction(
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD)
                        )
                ),
                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                //elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, downTime),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
        );

        Action pickup2 = new SequentialAction(
                new ParallelAction(
                        m_pickup2,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                )
        );

        Action floor_pickup1 = new SequentialAction(
                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_EXTENDED),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
        );

        Action wall_pickup1 = new ParallelAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.TAKE_SPECIMEN.state, 1100),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_AFTERSPIT),
                //sweeper in,
                m_pickup
        );

        Action pickup3 = new SequentialAction(
                new ParallelAction(
                        m_pickup3,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                )
        );


        Action hang2 = new SequentialAction(
                new ParallelAction(
                        m_hang2,
                        new SequentialAction(
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD)
                        )
                ),
                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                //elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, downTime),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
        );

        Action hang3 = new SequentialAction(
                new ParallelAction(
                        m_hang3,
                        new SequentialAction(
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD)
                        )
                ),
                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                //elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH_PRELOAD),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, downTime),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
        );



        Action park = new ParallelAction(
                m_park,
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
        );

        Action wall_pickup2 = new ParallelAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.TAKE_SPECIMEN.state, 1100),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_AFTERSPIT),
                //sweeper in,
                m_pickup2);

        Action wall_pickup3 = new ParallelAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.TAKE_SPECIMEN.state, 1100),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_AFTERSPIT),
                //sweeper in,
                m_pickup3);

        Action clawUpdate = claws.getUpdateClawAction(29);


        //initialization

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        clawUpdate,
                        new SequentialAction(
                                preload,
                                intake,
                                //floor_pickup1,

                                m_turn,
                                m_intake2,
                                sweeper.getSweeperAction(Sweeper.SweeperAngle.SWEEPER_EXTENDED,0),
                                wall_pickup1,
                                //TODO: pickup
                                new SequentialAction(
                                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY,75),
                                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                                ),
                                hang1,

                                wall_pickup2,
                                //TODO: pickup
                                new SequentialAction(
                                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY,75),
                                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)

                                ),
                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
                                hang2,

                                wall_pickup3,
                                //TODO: pickup
                                new SequentialAction(
                                        elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_HALFWAY,75),
                                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)

                                ),
                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
                                hang3


//
//                        new ParallelAction(
//                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_AFTERSPIT),
//                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED,1000),
//                                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT_HARD, colorSensorSystem)
//                                ),
//                        pickup2,
//                        new ParallelAction(
//                                hang2,
//                                actionControl.hangSpecimenHighOpMode()
//                        ),
//
//                        new ParallelAction(
//                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_AFTERSPIT),
//                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED,1000),
//                                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT_HARD, colorSensorSystem)
//                        ),
//                        pickup3,
//                        new ParallelAction(
//                                hang3,
//                                actionControl.hangSpecimenHigh()
//                        ),
//
//                        new ParallelAction(
//                                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_AFTERSPIT),
//                                elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED,1000),
//                                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT_HARD, colorSensorSystem)
//                        ),
//                        park
                        )
                )
        );
    }
}