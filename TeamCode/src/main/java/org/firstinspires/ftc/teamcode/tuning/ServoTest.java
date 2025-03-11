package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Elevators;
import org.firstinspires.ftc.teamcode.Systems.Sweeper;

@TeleOp(name = "ServoTest", group = "Tests")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Sweeper sweeper = new Sweeper(this);
        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.cross){
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_RETRACTED);
            }

            if(gamepad1.square){
                sweeper.setPosition(Sweeper.SweeperAngle.SWEEPER_EXTENDED);
            }


            telemetry.addData("pos:", sweeper.getPosition());
            telemetry.update();
        }
    }
}
