package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@Config
@TeleOp(name="ClawPIDFTuning", group = "Tests")
public class ClawPIDFTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = new DifferentialClaws(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();


        while (opModeIsActive()) {
            if(gamepad1.square) {
                claws.updateRightClawServoRotation();
                claws.updateLeftClawServoRotation();
                //double power = pid + ff;

                claws.rotateArm(claws.getPIDArmPower());
            }
            telemetry.addData("pos: ", claws.getActualArmRotation());
            telemetry.addData("target: ", claws.getArmTargetPosition());
            telemetry.addData("curr rotation", claws.getArmPosition());
            telemetry.addData("right claw", claws.getServoVirtualPosition()[1]);
            telemetry.addData("left claw", claws.getServoVirtualPosition()[0]);

            telemetry.update();

            if (gamepad1.cross) {
                claws.setArmTargetPosition(90);
            }
            else if (gamepad1.circle) {
                claws.setArmTargetPosition(0);
            }
            else if (gamepad1.triangle) {
                Actions.runBlocking(claws.setClawMovementAction(40));
            }
        }
    }
}
