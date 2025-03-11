package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Elevators.HorizontalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;
import org.firstinspires.ftc.teamcode.Systems.Token.Token;
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
    GamepadEx gamepad1, gamepad2;

    Sweeper sweeper;

    public ActionControl(Elevators elevators, DifferentialClaws claws, ColorSensorSystem colorSensorSystem,
                         MecanumDrive drive, Sweeper sweeper, GamepadEx gamepad1, GamepadEx gamepad2) {
        this.elevators = elevators;
        this.claws = claws;
        this.colorSensorSystem = colorSensorSystem;
        this.mecanumDrive = drive;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.sweeper = sweeper;
        //TODO: ADD CLAW MOVEMENTS TO THESE
    }

    public Action returnFromDrop(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state,1000, stopToken), // up
                //elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN, stopToken),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800, stopToken))
                , stopToken); // down
    }

    public Action returnWithDrive(TokenSequentialAction tokenAction, Token stopToken){
        return new ParallelAction(mecanumDrive.getMecanumDriveAction(gamepad1, gamepad2, sweeper, tokenAction, stopToken)
                , tokenAction);
    }

    public Action getReadyDropHigh(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750, stopToken), //up
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH, stopToken),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750, stopToken)
//                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,1000),
//                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750)
        ), stopToken);
    }

    public Action getReadyPickUpSpecimen(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MIN.state, 750, stopToken),
                new TokenParallelAction(
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.TAKE_IN, colorSensorSystem, stopToken),
                elevators.setHorizontalElevatorAction(HorizontalState.HORIZONTAL_HALFWAY, stopToken)
                )
        ), stopToken);
    }

    public  Action getReadyDropLow(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000),//mid
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW),
                elevators.setHorizontalElevatorAction(HorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ), stopToken);
    }

    public Action returnFromPickUp(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(45, 750), //mid
                elevators.setHorizontalElevatorAction(HorizontalState.HORIZONTAL_RETRACTED),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ), stopToken);
    }

    public Action getReadyExtendedPickUp(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(45, 750), //mid
                elevators.setHorizontalElevatorAction(HorizontalState.HORIZONTAL_EXTENDED)
                //claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ), stopToken);
    }

    public Action getReadyHalfwayPickUp(){
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(45, 750), //mid
                elevators.setHorizontalElevatorAction(HorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 800) // down
        ), stopToken);
    }

    public Action hangSpecimenHigh() {
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN)
                )
                , stopToken);
    }

    public Action hangHighRaise() {
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP)
                )
                , stopToken);
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
