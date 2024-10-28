package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Elevators {
    Servo verticalServo1;
    Servo verticalServo2;
    DcMotorEx horizontalMotor1;
    DcMotorEx horizontalMotor2;

    public class ElevatorAction extends Action {

    }

    enum ElevatorState {
        VERTICAL_ZERO(0),
        VERTICAL_LOW(1),
        VERTICAL_MEDIUM(2),
        VERTICAL_HIGH(3),
        VERTICAL_MAX(4),
        HORIZONTAL_RETRACTED(0),
        HORIZONTAL_EXTENDED(1);

        public final int state;

        ElevatorState(int state) {
            this.state = state;
        }
    }

    public Elevators(OpMode opMode) {

    }

    public void setVertical(int state) {
        
    }
}
