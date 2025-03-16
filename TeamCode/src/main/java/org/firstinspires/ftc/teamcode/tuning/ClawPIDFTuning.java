package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.Systems.DifferentialClaws;

@Config
@TeleOp(name="ClawPIDFTuning", group = "Tests")
public class ClawPIDFTuning extends LinearOpMode {
    public static double pos = 0;
    public static double p = 0.01, i = 0.0002, d = 0.0002;//0.0075, i=0,d=0.00005;//0.008, i = 0, d = 0.0001;
    public static double f = 0.07;//0.05;
    public static double x = 0;
    public static double startTime;

    @Override
    public void runOpMode() throws InterruptedException {
        DifferentialClaws claws = DifferentialClaws.getInstance(this);
        //MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ColorSensorSystem colorSensorSystem = new ColorSensorSystem(this, false);

        waitForStart();

        startTime = System.currentTimeMillis();
        boolean isDoneOnce = false;
        boolean isUp = false;
        double lastPIDPower = 0;
        while (opModeIsActive()) {
            if (pos < 0) {
                pos = 0;
            }
            else if (pos > DifferentialClaws.maxPoint) {
                pos = DifferentialClaws.maxPoint;
            }

            claws.controller.setPID(p, i, d);
            claws.setF(f);

            telemetry.addData("pos: ", claws.getActualArmRotation());
            telemetry.addData("actual pos", pos);
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
                x = System.currentTimeMillis() - startTime;
                x = x % 8000;

                if(!isDoneOnce) {
                    if (x <= 4000) {
                        Actions.runBlocking(claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750));
                        isUp = true;
                    } else {
                        isUp = false;
                        Actions.runBlocking(claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750));
                    }
                    isDoneOnce = true;
                }
                else
                    isDoneOnce = !((isUp || x<=4000) && !(isUp && x<=4000)); //XOR
            }

            if(gamepad2.circle)
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem));
            else if(gamepad2.cross)
                Actions.runBlocking(claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT, colorSensorSystem));
        }
    }
}


