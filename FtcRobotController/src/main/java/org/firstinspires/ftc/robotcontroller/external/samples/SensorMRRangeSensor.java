/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

import java.util.Dictionary;

/*
 * This OpMode illustrates how to use the Modern Robotics Range Sensor.
 *
 * The OpMode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@TeleOp(name = "Sensor: MR range sensor", group = "Sensor")
//@Disabled   // comment out or remove this line to enable this OpMode
public class SensorMRRangeSensor extends LinearOpMode {

    LynxI2cColorRangeSensor rangeSensor;
    enum SpecimenColor{
        RED, BLUE, YELLOW, NO_COLOR_DETECTED;
        public static SpecimenColor GetColor (double hue){
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
    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(LynxI2cColorRangeSensor.class, "sensor_range");

        // wait for the start button to be pressed
        waitForStart();
        double hue = 0;
        double maxColor;
        double minColor;
        double red;
        double green;
        double blue;
        while (opModeIsActive()) {
            red = rangeSensor.getNormalizedColors().red;
            green = rangeSensor.getNormalizedColors().green;
            blue = rangeSensor.getNormalizedColors().blue;

            telemetry.addData("green", green);
            telemetry.addData("red", red);
            telemetry.addData("blue", blue);
            telemetry.addData("alpha", rangeSensor.alpha());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("argb", rangeSensor.argb());

            maxColor = Math.max(Math.max(red, green), blue);
            minColor = Math.min(Math.min(red, green), blue);
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
            telemetry.addData("hue", hue);
            telemetry.addData("Color", SpecimenColor.GetColor(hue));
            telemetry.update();
        }
    }
}
