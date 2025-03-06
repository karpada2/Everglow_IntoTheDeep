package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Elevators;

@TeleOp(name = "ServoTest", group = "Tests")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo rightHor = hardwareMap.get(Servo.class, "rightHor");
        Servo leftHor = hardwareMap.get(Servo.class, "leftHor");
        rightHor.setDirection(Servo.Direction.REVERSE);
        leftHor.setDirection(Servo.Direction.FORWARD);

        CRServo right = hardwareMap.get(CRServo.class, "rightClawServo");
        waitForStart();

        double pos = 0;
        while (opModeIsActive()){
            if(gamepad1.cross){
                pos += 0.008;
                pos = Math.min(1, pos);
            }

            if(gamepad1.square){
                pos -= 0.008;
                pos = Math.max(0, pos);
            }
            right.setPower(-gamepad1.left_stick_y);
            leftHor.setPosition(pos);
            rightHor.setPosition(pos);

            telemetry.addData("right Hor:", rightHor.getPosition());
            telemetry.addData("left Hor:", leftHor.getPosition());
            telemetry.addData("pos:", pos);
            telemetry.update();
        }
    }
}
