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

    public Action returnWithDrive(TokenSequentialAction tokenAction, Token stopToken){
        return new ParallelAction(mecanumDrive.getMecanumDriveAction(gamepad1, gamepad2, sweeper, tokenAction, stopToken)
                , tokenAction);
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
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750, stopToken),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH, stopToken),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750, stopToken),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP, stopToken)
                )
                , stopToken);
    }

    public Action dropHigh() {
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH, stopToken),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750, stopToken),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem, stopToken),
                        new TokenParallelAction(
                                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW, stopToken),
                                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750, stopToken)
                        )
                )
                , stopToken);
    }

}
