package org.firstinspires.ftc.teamcode.EverglowLibrary.ThreadHandleLib;

import org.firstinspires.ftc.teamcode.Systems.Claws;
import org.firstinspires.ftc.teamcode.Systems.Elevators;


public class SequenceControl {

//    private boolean seq1_toggle = false;
//    private boolean seq2_toggle = false;
//    private boolean seq3_toggle = false;
//    private boolean seq4_toggle = false;
//    private boolean upAndDown_toggle = false;

    public final Sequence halfPickUpSeq;
    public final Sequence extendedPickUpSeq;
    public final Sequence returnFromPickUp;
    public final Sequence getReadyToDropHighSeq;
    public final Sequence getReadyToDropLowSeq;
    public final Sequence returnFromDrop;

    public SequenceControl(Elevators elevators, Claws claws){
         halfPickUpSeq = new Sequence(false
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_HURDLE)
                , elevators.getHorizontalExecutor(Elevators.HorizontalState.HORIZONTAL_HALFWAY,true)
                 , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_PICKUP));

        extendedPickUpSeq = new Sequence(false
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_HURDLE)
                , elevators.getHorizontalExecutor(Elevators.HorizontalState.HORIZONTAL_EXTENDED,true)
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_PICKUP));

        returnFromPickUp = new Sequence(false
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_HURDLE)
                , elevators.getHorizontalExecutor(Elevators.HorizontalState.HORIZONTAL_RETRACTED,true)
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_PICKUP));

        getReadyToDropHighSeq = new Sequence(true
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_HIGH)
                , elevators.getHorizontalExecutor(Elevators.HorizontalState.HORIZONTAL_HALFWAY, true));

        getReadyToDropLowSeq = new Sequence(true
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_LOW)
                , elevators.getHorizontalExecutor(Elevators.HorizontalState.HORIZONTAL_HALFWAY, true));

        returnFromDrop = new Sequence(false
                , elevators.getHorizontalExecutor(Elevators.HorizontalState.HORIZONTAL_RETRACTED,true)
                , elevators.getVerticalExecutor(Elevators.VerticalState.VERTICAL_PICKUP));
    }
}
