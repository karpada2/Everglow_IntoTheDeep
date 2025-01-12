package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Systems.Elevators.MotorHorizontalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;


public class ActionControl {
    public final Action getReadyExtendedPickUp;
    public final Action getReadyHalfwayPickUp;
    public final Action returnFromPickUp;
    public final Action getReadyDropLow;
    public final Action getReadyDropHigh;
    public final Action returnFromDrop;

    public ActionControl(Elevators elevators, DifferentialClaws claws) {
        getReadyHalfwayPickUp = new SequentialAction(
                claws.setClawMovementAction(90), //up
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.setClawMovementAction(0) // down
        );

        getReadyExtendedPickUp =  new SequentialAction(
                claws.setClawMovementAction(90), //up
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_EXTENDED),
                claws.setClawMovementAction(0) // down
        );

        returnFromPickUp = new SequentialAction(
                claws.setClawMovementAction(90), //up
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                claws.setClawMovementAction(0) // down
        );

        getReadyDropLow = new SequentialAction(
                claws.setClawMovementAction(90), //up
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW),
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.setClawMovementAction(0) // down
        );

        getReadyDropHigh = new SequentialAction(
                claws.setClawMovementAction(90), // up
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH),
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.setClawMovementAction(0) // down
        );

        returnFromDrop = new SequentialAction(
                claws.setClawMovementAction(90), // up
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN),
                claws.setClawMovementAction(0) // down
        );
        //TODO: ADD CLAW MOVEMENTS TO THESE
    }
}
