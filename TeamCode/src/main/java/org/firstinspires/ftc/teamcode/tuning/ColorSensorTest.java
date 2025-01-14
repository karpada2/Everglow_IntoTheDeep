package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;

@TeleOp(name = "ColorSensorTest")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, true);

        //ColorSensorSystem.SpecimenColor lastSpecimenColor = ColorSensorSystem.SpecimenColor.NO_COLOR_DETECTED;
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Distance:", colorSensorSystem.getDistance(DistanceUnit.CM));
            telemetry.addData("Color detected:", colorSensorSystem.getSpecimenColor());
            telemetry.addData("Claw contain specimen?", colorSensorSystem.isSpecimenIn());

            colorSensorSystem.updateAlert();
            telemetry.update();
        }
    }
}
