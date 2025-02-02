package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenSequentialAction;

@TeleOp(name = "TokenActionTest", group = "Tests")
public class TokenActionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = new Elevators(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        DifferentialClaws claws = new DifferentialClaws(this);

        waitForStart();

        TokenSequentialAction elevAction = new TokenSequentialAction(elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_HIGH),
                claws.clawMovementAction(260, 750),
                elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_LOW));

        Action runAction = new ParallelAction(
                drive.getMecanumDriveAction(gamepad1,gamepad2, elevAction),
                elevAction);

        while (opModeIsActive()){
            if(gamepad1.square){
                Actions.runBlocking(runAction);
            }
        }
    }
}
