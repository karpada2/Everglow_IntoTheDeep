package org.firstinspires.ftc.teamcode.Systems;

import android.widget.GridLayout;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorSystem {

    LynxI2cColorRangeSensor colorSensor;
    boolean isTeamBlue;

    Gamepad gamepad1;

    Gamepad gamepad2;

    private SpecimenColor lastSpecimenColor = SpecimenColor.NO_COLOR_DETECTED;

    public enum SpecimenColor{
        RED, BLUE, YELLOW, NO_COLOR_DETECTED;
        public static SpecimenColor getColor (double hue){
            int red = 0, yellow = 40, blue = 200;
            final double epsilon = 19;
            if (hue < red+epsilon) {
                return SpecimenColor.RED;
            }
            else if (blue-epsilon < hue && hue < blue+epsilon) {
                return SpecimenColor.BLUE;
            }
            else if (yellow-epsilon < hue && hue < yellow+epsilon) {
                return SpecimenColor.YELLOW;
            }
            else return NO_COLOR_DETECTED;
        }
    }
    public ColorSensorSystem(OpMode opMode, boolean isBlueTeam){
        colorSensor = opMode.hardwareMap.get(LynxI2cColorRangeSensor.class, "clawSensor");
        isTeamBlue = isBlueTeam;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        colorSensor.enableLed(true);
    }

    public boolean isSpecimenIn(){
        double minValue = 5.5; //cm
        return colorSensor.getDistance(DistanceUnit.CM) <= minValue;
    }

    public double getDistance(DistanceUnit unit){
        return  colorSensor.getDistance(unit);
    }
    public double getHue() {
        double red = colorSensor.getNormalizedColors().red;
        double green = colorSensor.getNormalizedColors().green;
        double blue = colorSensor.getNormalizedColors().blue;
        double maxColor = Math.max(Math.max(red, green), blue);
        double minColor = Math.min(Math.min(red, green), blue);
        double hue = 0;
        if (maxColor == red){
            hue = (green-blue)/(maxColor-minColor);
        }
        else if (maxColor == green) {
            hue = 2.0 + (blue - green) / (maxColor - minColor);
        }
        else if (maxColor == blue){
            hue = 4.0 + (red-green)/(maxColor-minColor);
        }
        hue *= 60;
        if (hue < 0) hue+=360;

        return hue;
    }
    public SpecimenColor getSpecimenColor(){
        double hue = getHue();
        if (isSpecimenIn())
            return SpecimenColor.getColor(hue);
        else
            return SpecimenColor.NO_COLOR_DETECTED;
    }

    public boolean myTeamSpecimen(SpecimenColor specimenColor){
        return (specimenColor == SpecimenColor.BLUE && isTeamBlue) || (specimenColor == SpecimenColor.RED && !isTeamBlue)
                || (specimenColor == SpecimenColor.YELLOW);
    }
    public void alertToGamePads(){
        final int rightColorFreq = 210, badColorFreq = 750; //milliseconds
        Gamepad.RumbleEffect rumbleEffect;
        SpecimenColor specimenColor = getSpecimenColor();
        if (specimenColor == SpecimenColor.NO_COLOR_DETECTED){
            gamepad1.stopRumble();
            gamepad2.stopRumble();
            return;
        }

        int freqTime = 0;
        int timeWait = 0;
        double power = 0;
        if(myTeamSpecimen(specimenColor)) {
            freqTime = rightColorFreq; //right specimen
            timeWait = 300;
            power = 1;
        }
        else {  //not good specimen
            freqTime = badColorFreq;
            timeWait = 510;
            power = 0.9;
        }

        if(!gamepad1.isRumbling()) {
            rumbleEffect = new Gamepad.RumbleEffect.Builder()
                    .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                    .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                    .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                    .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                    .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                    .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                    .build();

            gamepad1.runRumbleEffect(rumbleEffect);
            gamepad2.runRumbleEffect(rumbleEffect);
        }
    }

    public void updateAlert(){
        if(lastSpecimenColor != getSpecimenColor() || !myTeamSpecimen(getSpecimenColor())){
            alertToGamePads();
        }

        lastSpecimenColor = getSpecimenColor();
    }
}
