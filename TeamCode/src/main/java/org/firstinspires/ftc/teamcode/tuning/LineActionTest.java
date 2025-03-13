package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.ActionControl;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Sweeper;

@TeleOp(name = "LineActionTest", group = "Tests")
public class LineActionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = Elevators.getInstance(this);
        DifferentialClaws differentialClaws = DifferentialClaws.getInstance(this);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);
        Sweeper sweeper = new Sweeper(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-31.1, -63,   Math.PI));

        GamepadEx gamepadEx1 = new GamepadEx(this.gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(this.gamepad2);

        ActionControl actionControl = new ActionControl(elevators, differentialClaws, colorSensorSystem, drive,
                sweeper, gamepadEx1, gamepadEx2);

        waitForStart();

        while (opModeIsActive()){
            gamepadEx2.readButtons();
            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.A)){
                Actions.runBlocking(actionControl.splineToDropLine());
                Actions.runBlocking(actionControl.dropHighAndToPlace());
            }
        }
    }
}
