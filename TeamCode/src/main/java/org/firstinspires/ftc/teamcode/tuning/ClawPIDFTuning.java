package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@Config
@TeleOp(name="ClawPIDFTuning", group = "Tests")
public class ClawPIDFTuning extends LinearOpMode {
    public static double p = 0, i = 0, d = 0;//0.0075, i=0,d=0.00005;//0.008, i = 0, d = 0.0001;
    public static double f = 0;//0.05;
    public static double x = 0;
    public static double startTime;

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = DifferentialClaws.getInstance(this);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        startTime = System.currentTimeMillis();
        double lastPIDPower = 0;
        while (opModeIsActive()) {
            claws.controller.setPID(p, i, d);
            claws.setF(f);

            telemetry.addData("pos: ", claws.getActualArmRotation());
            telemetry.addData("target: ", claws.getArmTargetPosition());
            telemetry.addData("curr rotation", claws.getArmPosition());
            //telemetry.addData("curr rotation", claws.getArmPosition());
            telemetry.addData("right claw", claws.getServoVirtualPosition()[1]);
            telemetry.addData("left claw", claws.getServoVirtualPosition()[0]);
            telemetry.addData("power: ", lastPIDPower);


            claws.updateRightClawServoRotation();
            claws.updateLeftClawServoRotation();
            telemetry.update();

            if (gamepad2.square) {
                lastPIDPower = claws.getPIDArmPower();
                claws.rotateArm(lastPIDPower);
                x = System.currentTimeMillis() - startTime;
                x = x % 8000;

                if (x <= 4000) {
                    claws.setArmTargetPosition(68);
                } else {
                    claws.setArmTargetPosition(48);
                }
            }
        }
    }
}


