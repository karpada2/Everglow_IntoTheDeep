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
        Pose2d hangPose = new Pose2d(6,-32,Math.PI/2);;
        double VelConstraint = 20;

        double firstSpeciminX = 44;
        double secondSpeciminX = 56;
        double PickSpeciminY = -42;
        double dropSpeciminY = -52;

        Pose2d specimins_basketPose = new Pose2d(0,-36,Math.PI/2);
        Pose2d specimins_basketPose2 = new Pose2d(-2,-36,Math.PI/2);
        Pose2d specimins_pickupPose = new Pose2d(48,-52,-Math.PI/2);



        // Init Systems
        DifferentialClaws claws  = DifferentialClaws.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, specimins_beginPose);
        Elevators elevators  = Elevators.getInstance(this);

        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        ActionControl actionControl = new ActionControl(elevators,claws,colorSensorSystem,drive, new Sweeper(this),new GamepadEx(gamepad1), new GamepadEx(gamepad2));
        //Init Trajectories


        TrajectoryActionBuilder B_preload = drive.actionBuilder(specimins_beginPose)
                //starts at hanging height
                .strafeTo(specimins_basketPose.position)
                //* Hangs
                ;

        TrajectoryActionBuilder B_push = B_preload.endTrajectory().fresh()
                .setTangent(-Math.PI/4)
                .splineToConstantHeading(new Vector2d(33,-30),Math.PI/2)
                .splineToSplineHeading(new Pose2d(firstSpeciminX-10,PickSpeciminY,Math.PI),0)
                .splineToConstantHeading(new Vector2d(firstSpeciminX,dropSpeciminY),-Math.PI/2)
                // Moves to the second specimin
                .splineToConstantHeading(new Vector2d(secondSpeciminX,PickSpeciminY),0)
                // Waits for human player
                // Pushes the second specimin
                .splineToConstantHeading(new Vector2d(55,-45),Math.PI/1.6)
                .splineToSplineHeading(specimins_pickupPose,-Math.PI/2)
                ;


        TrajectoryActionBuilder B_hang1 = B_push.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0,-54), specimins_basketPose.heading)
                .strafeTo(specimins_basketPose.position, new TranslationalVelConstraint(150))
                ;

        TrajectoryActionBuilder B_pickup2 = B_hang1.endTrajectory().fresh()
                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                ;

        TrajectoryActionBuilder B_hang2 = B_pickup2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0,-54), specimins_basketPose.heading)
                .strafeTo(specimins_basketPose.position, new TranslationalVelConstraint(150))
                ;

        TrajectoryActionBuilder B_pickup3 = B_hang2.endTrajectory().fresh()
                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                ;

        TrajectoryActionBuilder B_hang3 = B_pickup2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(0,-54), specimins_basketPose.heading)
                .strafeTo(specimins_basketPose.position, new TranslationalVelConstraint(150))
                ;



        TrajectoryActionBuilder B_park = B_pickup2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI)
                ;



        Action m_preload = B_preload.build();

        Action m_push = B_push.build();
        Action m_hang1 = B_hang1.build();

        Action m_pickup2 = B_pickup2.build();

        Action m_park = B_park.build();

        Action preload = new SequentialAction(
                new ParallelAction(
                        m_preload, //movement
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH)
                ),
                actionControl.hangHighRaise()
        );

        Action push = new SequentialAction(
                new ParallelAction(
                        m_push,
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        new SequentialAction(
                                new WaitAction(5000),
                                new ParallelAction(
                                    claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 800),
                                    elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                                ),
                                new ParallelAction(
                                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                                )
                        )
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 800),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem)
                )
        );

        Action hang1 = new SequentialAction(
                new ParallelAction(
                        m_hang1
                ),
                actionControl.hangHighRaise()
        );

        Action pickup2 = new SequentialAction(
                new ParallelAction(
                        m_pickup2,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MID.state, 750),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 200)
                )
        );
        Action hang2 = new SequentialAction(
                new ParallelAction(
                        m_hang2
                ),
                actionControl.hangHighRaise()
        );

        Action pickup3 = new SequentialAction(
                new ParallelAction(
                        m_pickup3,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                ),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MID.state, 750),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 200)
                )
        );


        Action park = new ParallelAction(
                m_park,
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
        );


        //initialization

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        pickup1,
                        actionControl.hangSpecimenHigh(),
                        hang1,
                        pickup2,
                        park
                )
        );
    }
}