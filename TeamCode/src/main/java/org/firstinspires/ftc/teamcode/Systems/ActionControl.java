package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Elevators.MotorHorizontalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenAction;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenParallelAction;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenSequentialAction;


public class ActionControl {
//    public final Action getReadyExtendedPickUp;
//    public final Action getReadyHalfwayPickUp;
//    public final Action returnFromPickUp;
//    public final Action getReadyDropLow;
//    public final Action getReadyDropHigh;
//    public final Action returnFromDrop;
    Elevators elevators;
    DifferentialClaws claws;

    ColorSensorSystem colorSensorSystem;
    MecanumDrive mecanumDrive;
    Gamepad gamepad1, gamepad2;


    private boolean isRunAction = false;
    private Thread runingThread;

    public ActionControl(Elevators elevators, DifferentialClaws claws, ColorSensorSystem colorSensorSystem,
                         MecanumDrive drive, Gamepad gamepad1, Gamepad gamepad2) {
        this.elevators = elevators;
        this.claws = claws;
        this.colorSensorSystem = colorSensorSystem;
        this.mecanumDrive = drive;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        //TODO: ADD CLAW MOVEMENTS TO THESE
    }

    public Action returnFromDrop(){

        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state,1000), // up
                //elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800))); // down
    }

    public Action returnWithDrive(TokenSequentialAction tokenAction){
        return new ParallelAction(mecanumDrive.getMecanumDriveAction(gamepad1, gamepad2, tokenAction), tokenAction);
    }

    public Action getReadyDropHigh(){
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750), //up
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // ,mid
                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state) // down
        ));
    }

    public Action getReadyPickUpSpecimen(){
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750),
                new TokenParallelAction(
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem),
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY)
                )
        ));
    }

    public  Action getReadyDropLow(){
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),//mid
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW),
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ));
    }

    public Action returnFromPickUp(){
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(45, 750), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ));
    }

    public Action getReadyExtendedPickUp(){
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(45, 750), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_EXTENDED)
                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ));
    }

    public Action getReadyHalfwayPickUp(){
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(45, 750), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ));
    }

    public Action hangSpecimenHigh() {
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN)
                )
        );
    }


//    public void runAction(Action action){
//        if(!isRunAction){
//            isRunAction = true;
//            runingThread = new Thread(() -> {
//                Actions.runBlocking(action);
//                isRunAction = false;
//            });
//            runingThread.start();
//        }
//    }
//
//    public boolean isOnRun(){
//        return  isRunAction;
//    }
}
