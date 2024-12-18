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
    public final Action reutrnFromDrop;

    public ActionControl(Elevators elevators, DifferentialClaws claws) {
        getReadyHalfwayPickUp = new SequentialAction(
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HURDLE),
                new SequentialAction(
                        elevators.setHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY.state),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_PICKUP)
                )
        );

        getReadyExtendedPickUp =  new SequentialAction(
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HURDLE),
                new SequentialAction(
                        elevators.setHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_EXTENDED.state),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_PICKUP)
                )
        );

        returnFromPickUp = new SequentialAction(
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HURDLE),
                new SequentialAction(
                        elevators.setHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED.state),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_PICKUP)
                )
        );

        getReadyDropLow = new SequentialAction(
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW),
                elevators.setHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY.state)
        );

        getReadyDropHigh = new SequentialAction(
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH),
                elevators.setHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY.state)
        );

        reutrnFromDrop = new SequentialAction(
                elevators.setHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED.state),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN)
        );
        //TODO: ADD CLAW MOVEMENTS TO THESE
    }
}
