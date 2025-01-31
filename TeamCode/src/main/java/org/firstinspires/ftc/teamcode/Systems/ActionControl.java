package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Systems.Elevators.MotorHorizontalState;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;


public class ActionControl {
    Elevators elevators;
    DifferentialClaws claws;

    private boolean isRunAction = false;
    private Thread runingThread;

    public ActionControl(Elevators elevators, DifferentialClaws claws) {
        this.elevators = elevators;
        this.claws = claws;
    }

    public Action returnFromDrop(){
        return new SequentialAction(
                claws.clawMovementAction(250,1000), // up
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN),
                claws.clawMovementAction(0, 500) // down
        );
    }

    public Action getReadyDropHigh(){
        return new SequentialAction(
                claws.clawMovementAction(250, 1000), //up
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH),
                claws.clawMovementAction(150, 500), // ,mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY)
                //claws.clawMovementAction(0) // down
        );
    }

    public  Action getReadyDropLow(){
        return new SequentialAction(
                claws.clawMovementAction(250, 1000),//mid
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_LOW),
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(0, 500) // down
        );
    }

    public Action returnFromPickUp(){
        return new SequentialAction(
                claws.clawMovementAction(170, 750), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_RETRACTED),
                claws.clawMovementAction(0, 500) // down
        );
    }

    public Action getReadyExtendedPickUp(){
        return new SequentialAction(
                claws.clawMovementAction(170, 750), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_EXTENDED),
                claws.clawMovementAction(0, 500) // down
        );
    }

    public Action getReadyHalfwayPickUp(){
        return new SequentialAction(
                claws.clawMovementAction(170, 750), //mid
                elevators.setMotorHorizontalElevatorAction(MotorHorizontalState.HORIZONTAL_HALFWAY),
                claws.clawMovementAction(0, 500) // down
        );
    }

    public Action hangSpecimenHigh() {
        return new SequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH),
                new ParallelAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN)
                )
        );
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
