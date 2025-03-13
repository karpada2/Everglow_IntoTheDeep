package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;

@TeleOp(name = "ColorSensorTest", group = "Tests")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LynxI2cColorRangeSensor colorRangeSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "clawSensor");
        ColorSensorSystem colorSystem = new ColorSensorSystem(this, true);
        int loopsDone = 0;
        double startTime = System.currentTimeMillis();
        double timeSinceStart= System.currentTimeMillis() - startTime;

        int freqTime = 210; //right specimen
        int timeWait = 300;
        double power = 1;
        Gamepad.RumbleEffect rumbleEffectGood = new Gamepad.RumbleEffect.Builder()
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .build();

        //ColorSensorSystem.SpecimenColor lastSpecimenColor = ColorSensorSystem.SpecimenColor.NO_COLOR_DETECTED;
        waitForStart();

//        NormalizedRGBA normalizedRGBA;
        ColorSensorSystem.SpecimenColor color;

        while(opModeIsActive()){
            loopsDone++;
            timeSinceStart = (System.currentTimeMillis() - startTime)/1000.0;
            //color = colorSystem.getSpecimenColor();
            colorSystem.updateAlert();
//            gamepad1.runRumbleEffect(rumbleEffectGood);
//            gamepad2.runRumbleEffect(rumbleEffectGood);
            //gamepad1.stopRumble();
            //gamepad2.stopRumble();
//            double red = normalizedRGBA.red;
//            double green = normalizedRGBA.green;
//            double blue = normalizedRGBA.blue;
//            double maxColor = Math.max(Math.max(red, green), blue);
//            double minColor = Math.min(Math.min(red, green), blue);
//            double hue = 0;
//            if (maxColor == red){
//                hue = (green-blue)/(maxColor-minColor);
//            }
//            else if (maxColor == green) {
//                hue = 2.0 + (blue - green) / (maxColor - minColor);
//            }
//            else if (maxColor == blue){
//                hue = 4.0 + (red-green)/(maxColor-minColor);
//            }
//            hue *= 60;
//            if (hue < 0) hue+=360;
//
//            telemetry.addData("distance", colorRangeSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("red", normalizedRGBA.red);
//            telemetry.addData("green", normalizedRGBA.green);
//            telemetry.addData("blue", normalizedRGBA.blue);
//            telemetry.addData("alpha", normalizedRGBA.alpha);
//            telemetry.addData("hue", hue);
            if (!gamepad1.cross) {
                telemetry.addData("color", colorSystem.getSpecimenColor());
//            telemetry.addData("red", colorSystem.leftSensor.red()); //280
//            telemetry.addData("green", colorSystem.leftSensor.green()); //555
//            telemetry.addData("blue", colorSystem.leftSensor.blue()); //570
//            telemetry.addData("alpha", colorSystem.leftSensor.alpha()); //470
                telemetry.addData("myColor", Integer.toHexString(colorSystem.leftSensor.argb())); //c7010202
                telemetry.addData("loops per second avg", loopsDone/timeSinceStart);
            }


            //yellow: Red: 0.31, Green: 0.282, Blue: 0.118, Alpha: 0.727
            //red: Red: 0.196, Green: 0.069, Blue: 0.0624, Alpha: 0.318
            //blue: Red: 0.048, Green: 0.085, Blue: 0.126, Alpha: 0.273
            //None: distance is greater than 10
            telemetry.update();
        }
    }
}
