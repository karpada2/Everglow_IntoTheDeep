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
        double secondSpeciminX = 56;
        double thirdSpeciminX = 60;
        double PickSpeciminY = -12;

        Pose2d specimins_basketPose = new Pose2d(0,-34,Math.PI/2);
        Pose2d specimins_endPose = new Pose2d(23,-10,0);
        Pose2d specimins_pickupPose = new Pose2d(48,-55,-Math.PI/2);
        // Init Systems
        DifferentialClaws claws  = DifferentialClaws.getInstance(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, specimins_beginPose);
        Elevators elevators  = Elevators.getInstance(this);

        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        ActionControl actionControl = new ActionControl(elevators,claws,colorSensorSystem,drive,gamepad1,gamepad2);
        //Init Trajectories


        TrajectoryActionBuilder B_preload = drive.actionBuilder(specimins_beginPose)
                //starts at hanging height
                .strafeTo(specimins_basketPose.position)
                //* Hangs
                ;


        TrajectoryActionBuilder B_push = B_preload.endTrajectory().fresh()
                // Goes to basket
                // Pushes Both Specimins into place
                .setTangent(-Math.PI/4)
                .splineToConstantHeading(new Vector2d(33,-33),Math.PI/2)
                .splineToSplineHeading(new Pose2d(firstSpeciminX-10,PickSpeciminY,Math.PI),0)
                .splineToConstantHeading(new Vector2d(firstSpeciminX,dropSpeciminY),-Math.PI/2)
                // Pushes second specimen
                .splineToConstantHeading(new Vector2d(secondSpeciminX,PickSpeciminY),0)
                .splineToConstantHeading(new Vector2d(secondSpeciminX,dropSpeciminY),-Math.PI/2)
                // Pushes third specimen
                .splineToConstantHeading(new Vector2d(thirdSpeciminX,PickSpeciminY),0)
                .splineToConstantHeading(new Vector2d(thirdSpeciminX,dropSpeciminY),-Math.PI/2)
                ;

        TrajectoryActionBuilder B_pickup1 = B_push.endTrajectory().fresh()
                //* Raises Elevator to picking up height
                // Goes to pickup first specimin
                .setTangent(Math.PI/2)
                .splineToLinearHeading(new Pose2d(specimins_pickupPose.position,-Math.PI/2),-Math.PI/2)
                //* Lowers elevator to pickup
                ;

        TrajectoryActionBuilder B_hang1 = B_pickup1.endTrajectory().fresh()
                // Goes to basket
                // *Raises elevator for hang
                .setTangent((1) * Math.PI)
                .strafeToSplineHeading(specimins_basketPose.position, specimins_basketPose.heading)
                //* Hangs Specimen sequentially
                ;

        TrajectoryActionBuilder B_pickup2 = B_hang1.endTrajectory().fresh()
                //* Lowers Elevator to picking up height
                // Goes to pickup second specimen
                .strafeToLinearHeading(specimins_pickupPose.position,-Math.PI/2)
                //* Picks up second specimen
                ;

        TrajectoryActionBuilder B_hang2 = B_pickup2.endTrajectory().fresh()
                // Goes to hang Specimen
                .strafeToLinearHeading(specimins_basketPose.position,Math.PI/2)
                //* Hangs
                ;

        TrajectoryActionBuilder B_park = B_pickup2.endTrajectory().fresh()
                //*Lowers elevator
                // Park
                .setTangent(-Math.PI/4)
                .splineToSplineHeading(new Pose2d(60,-60,Math.PI),0);


        Action m_preload = B_preload.build();

        Action m_push = B_push.build();

        Action m_pickup1 = B_pickup1.build();
        Action m_hang1 = B_hang1.build();

        Action m_pickup2 = B_pickup2.build();
        Action m_hang2 = B_hang2.build();

        Action m_park = B_park.build();

        Action preload   = new SequentialAction(
                new ParallelAction(
                        m_preload,
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH)
                        ),
                actionControl.hangHighRaise()
        );

        Action push   = new ParallelAction(
                m_push,
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750)
        );

        Action pickup1 = new SequentialAction(
                new ParallelAction(
                    m_pickup1,
                    elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750)
        ),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 200)
                )
        );

        Action hang1 = new SequentialAction(
                new ParallelAction(
                        m_hang1,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH)
                        ),
                actionControl.hangHighRaise()
        );

        Action pickup2 = new SequentialAction(
                new ParallelAction(
                        m_pickup2,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                ),
                new ParallelAction(
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, 200)
                )
        );

        Action hang2 = new SequentialAction(
                new ParallelAction(
                        m_hang2,
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_HIGH)
                ),
                actionControl.hangHighRaise()
        );

        Action park = new ParallelAction(
                m_park,
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN)
        );


        //initialization

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        push,
                        pickup1,
                        hang1,
                        pickup2,
                        hang2,
                        park
                )
        );
    }
}
