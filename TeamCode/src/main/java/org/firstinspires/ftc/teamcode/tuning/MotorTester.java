package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "Motor test", group="Tests")
public class MotorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws differentialClaws = DifferentialClaws.getInstance(this);
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);

        waitForStart();

        while (opModeIsActive()) {
            //horPosition += -gamepad2.left_stick_x*10;
            if(gamepad2.square)
                Actions.runBlocking(differentialClaws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem));
            differentialClaws.rotateWheels(DifferentialClaws.ClawPowerState.OFF);
            telemetry.addData("Distance:", colorSensorSystem.getDistance(DistanceUnit.CM));
            telemetry.addData("Color detected:", colorSensorSystem.getSpecimenColor());
            telemetry.addData("Claw contain specimen?", colorSensorSystem.isSpecimenIn());

            colorSensorSystem.updateAlert();
            telemetry.update();
        }
    }
}
