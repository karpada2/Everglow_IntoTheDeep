package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
@Autonomous(name="OnePlusOneRightPath", group="Autonomous")
public class OnePlusOneRightPath extends LinearOpMode {

    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses
        Pose2d specimins_beginPose = new Pose2d(16, -62,   Math.PI/2);
        Pose2d hangPose = new Pose2d(6,-32,Math.PI/2);;
        double VelConstraint = 20;

        double firstSpeciminX = 44;
        double PickSpeciminY = -42;
        double secondSpeciminX = 56;

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



        TrajectoryActionBuilder B_pickup1 = B_preload.endTrajectory().fresh()
                .strafeToSplineHeading(specimins_pickupPose.position,-Math.PI/2)
                ;

        TrajectoryActionBuilder B_hang1 = B_pickup1.endTrajectory().fresh()
                .strafeToSplineHeading(specimins_basketPose2.position,Math.PI/2)
                ;

        TrajectoryActionBuilder B_pickup2 = B_hang1.endTrajectory().fresh()
                .setTangent(-Math.PI/4)
                .splineToConstantHeading(new Vector2d(firstSpeciminX,PickSpeciminY), Math.PI/2)
                ;



        TrajectoryActionBuilder B_park = B_pickup2.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(60,-60),Math.PI)
                ;

        TrajectoryActionBuilder B_readyToHang1 = B_pickup1.endTrajectory().fresh()
                .setTangent(Math.PI)
                .splineToSplineHeading(new Pose2d(0, -58, 0.5*Math.PI), Math.PI);


        Action m_preload = B_preload.build();

        Action m_pickup1 = B_pickup1.build();
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

        Action pickup1 = new SequentialAction(
                new ParallelAction(
                    m_pickup1,
                    elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP),
                    claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750)
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