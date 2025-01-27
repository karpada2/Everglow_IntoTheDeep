package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@Config
@TeleOp(name="ClawPIDFTuning")
public class ClawPIDFTuning extends LinearOpMode {
    public static double p =0,// 0.0075,
            i = 0,//0.001,
            d =0;// 0.00015;
    public static double f = 0;
    public static double x = 0;
    public static double startTime;

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        startTime = System.currentTimeMillis();
        double lastPIDPower = 0;
        while (opModeIsActive()) {
            claws.controller.setPID(p, i, d);
            claws.setF(f);

            telemetry.addData("pos: ", claws.getActualArmRotation());
            telemetry.addData("target: ", claws.getArmTargetPosition());
            //telemetry.addData("curr rotation", claws.getArmPosition());
            telemetry.addData("right claw", claws.getServoVirtualPosition()[1]);
            telemetry.addData("left claw", claws.getServoVirtualPosition()[0]);
            telemetry.addData("power: ", lastPIDPower);


            claws.updateRightClawServoRotation();
            claws.updateLeftClawServoRotation();
            telemetry.update();

            lastPIDPower = claws.getPIDArmPower();
            claws.rotateArm(lastPIDPower);
            x = System.currentTimeMillis() - startTime;
            x = x % 8000;

            if (x <= 4000) {
                claws.setArmTargetPosition(40);
            } else {
                claws.setArmTargetPosition(60);
            }


            if (gamepad1.cross) {
                claws.setArmTargetPosition(90);
            }
            else if (gamepad1.circle) {
                claws.setArmTargetPosition(0);
            }
            else if (gamepad1.triangle) {
                Actions.runBlocking(claws.clawMovementAction(40));
            }
        }
    }
}
