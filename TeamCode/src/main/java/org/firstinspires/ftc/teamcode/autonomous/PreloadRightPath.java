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
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@Config
@Autonomous(name="PreloadRightPath", group="Autonomous")
public class PreloadRightPath extends LinearOpMode {

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
    public static double BasketY = -34;
    public static double BasketHeading = Math.PI/2;
    public static double VelConstraint = 10;

    @Override
    public void runOpMode()  throws InterruptedException{
        // Init Poses

        Pose2d beginPose = new Pose2d(31.1, -63,   Math.PI/2);
        Pose2d dropPose = new Pose2d(31.1, -57,   0);
        Pose2d endPose = new Pose2d(63,-60,Math.PI/2);

        // Init Systems
        DifferentialClaws claws  = new DifferentialClaws(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Elevators elevators  = new Elevators(this);
        //Init Trajectories
        TrajectoryActionBuilder B_preload_1 = drive.actionBuilder(beginPose)
                .strafeToSplineHeading(new Vector2d(-5,BasketY),BasketHeading);

        TrajectoryActionBuilder B_preload_2 = B_preload_1.endTrajectory().fresh()
                .lineToY(BasketY+1);

        TrajectoryActionBuilder B_sample1pickup = B_preload_2.endTrajectory().fresh()
                .strafeToSplineHeading(dropPose.position, dropPose.heading);

        TrajectoryActionBuilder B_sample1basket_1 = B_sample1pickup.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-5,BasketY),BasketHeading);

        TrajectoryActionBuilder B_sample1basket_2 = B_sample1basket_1.endTrajectory().fresh()
                .lineToY(BasketY+1);

        TrajectoryActionBuilder B_Park = B_sample1basket_2.endTrajectory().fresh()
                .strafeToSplineHeading(endPose.position,endPose.heading);

        Action wait = drive.actionBuilder(new Pose2d(0,0,0))
                .waitSeconds(3)
                .build();

        Action release = new SequentialAction(
                claws.clawMovementAction(100),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SUB_HURDLE),
                new ParallelAction(
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN,1000),
                        claws.clawMovementAction(90)//
                ),
                new ParallelAction(
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,1000),
                        claws.clawMovementAction(80)
                ),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HURDLE)
        );

        Action preload_1 = B_preload_1.build();
        Action preload_2 = B_preload_2.build();

        Action sample1pickup = B_sample1pickup.build();
        Action sample1basket_1 = B_sample1basket_1.build();
        Action sample1basket_2 = B_sample1basket_2.build();

        Action Park = B_Park.build();


        Action unload1 = new ParallelAction(
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HURDLE),
                preload_1
        );
        Action unload2 = new SequentialAction(
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HURDLE),
                sample1basket_1
        );

        Action pickup1 = new ParallelAction(
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                sample1pickup
                );

        Action Park_Lower = new ParallelAction(
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_MIN),
                Park
        );
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        unload1,
                        preload_2,
                        release,
                        pickup1,
                        unload2,
                        sample1basket_2,
                        release,
                        Park_Lower
                )
        );
    }
}
