/*
--------------------------------------------------------------------------
| This is going to work by internally keeping each servo's rotation in   |
| degrees (0 is starting) and possibly the entire claw's position.       |
| Actions must be used so that the servo actually reaches it's position. |
--------------------------------------------------------------------------
*/

package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

public class DifferentialClaws {
    CRServo clawServo1;
    CRServo clawServo2;
    AnalogInput clawInput1;
    AnalogInput clawInput2;

    public class ClawMovementAction implements Action {
        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public class ClawSampleInteractionAction implements Action {
        @Override
        public void preview(@NonNull Canvas fieldOverlay) {
            Action.super.preview(fieldOverlay);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

}
