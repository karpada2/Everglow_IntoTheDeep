package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
import org.firstinspires.ftc.teamcode.Systems.Token.Token;

@TeleOp(name = "LineActionTest", group = "Tests")
public class LineActionTest extends LinearOpMode {
    boolean toggle = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = Elevators.getInstance(this);
        DifferentialClaws differentialClaws = DifferentialClaws.getInstance(this);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, false);
        Sweeper sweeper = new Sweeper(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-24,-11.7, 0));

        Action myAction = drive.getDriveUntilStopAction(colorSensorSystem, new Token());

        GamepadEx gamepadEx1 = new GamepadEx(this.gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(this.gamepad2);

        ActionControl actionControl = new ActionControl(elevators, differentialClaws, colorSensorSystem, drive,
                sweeper, gamepadEx1, gamepadEx2);

        waitForStart();

        while (opModeIsActive()){
            gamepadEx2.readButtons();
            if(gamepadEx2.wasJustPressed(GamepadKeys.Button.A)){
                Actions.runBlocking(
                        new SequentialAction(
                                actionControl.splineToDropLine(),
                                actionControl.dropHighAndToPlace()
                        )
                );
            }
            else if (gamepadEx2.wasJustPressed(GamepadKeys.Button.B)) {
                Actions.runBlocking(drive.getDriveUntilStopAction(colorSensorSystem, new Token(), telemetry));
            }
        }

        telemetry.update();
    }
}
