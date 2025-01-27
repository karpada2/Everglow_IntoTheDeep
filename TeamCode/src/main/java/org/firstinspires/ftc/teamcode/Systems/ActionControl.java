package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Systems.Elevators.MotorHorizontalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;


public class ActionControl {
    public final Action getReadyExtendedPickUp;
    public final Action getReadyHalfwayPickUp;
    public final Action returnFromPickUp;
    public final Action getReadyDropLow;
    public final Action getReadyDropHigh;
    public final Action returnFromDrop;

    private boolean isRunAction = false;
    private Thread runingThread;

    public ActionControl(Elevators elevators, DifferentialClaws claws) {
        getReadyHalfwayPickUp = new SequentialAction(
                claws.setClawMovementAction(50), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.setClawMovementAction(20) // down
        );

        getReadyExtendedPickUp = new SequentialAction(
                claws.setClawMovementAction(50), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_EXTENDED),
                claws.setClawMovementAction(0) // down
        );

        returnFromPickUp = new SequentialAction(
                claws.setClawMovementAction(50), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                claws.setClawMovementAction(30),
                claws.setClawMovementAction(0) // down
        );

        getReadyDropLow = new SequentialAction(
                claws.setClawMovementAction(100),//mid
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW),
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.setClawMovementAction(50) // down
        );

        getReadyDropHigh = new SequentialAction(
                claws.setClawMovementAction(100), //up
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH),
                claws.setClawMovementAction(50), // ,mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY)
                //claws.setClawMovementAction(0) // down
        );

        returnFromDrop = new SequentialAction(
                claws.setClawMovementAction(100), // up
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                claws.setClawMovementAction(40), // down
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN),
                claws.setClawMovementAction(0) // down
        );
        //TODO: ADD CLAW MOVEMENTS TO THESE
    }

    public void runAction(Action action){
        if(!isRunAction){
            isRunAction = true;
            runingThread = new Thread(() -> {
                Actions.runBlocking(action);
                isRunAction = false;
            });
            runingThread.start();
        }
    }

    public boolean isOnRun(){
        return  isRunAction;
    }
}
