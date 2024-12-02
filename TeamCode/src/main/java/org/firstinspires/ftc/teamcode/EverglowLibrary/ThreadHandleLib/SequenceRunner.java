package org.firstinspires.ftc.teamcode.EverglowLibrary.ThreadHandleLib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.EverglowLibrary.Systems.Executor;

import java.util.Calendar;

public class SequenceRunner {
    private boolean isFinished = true;
    private Sequence runningSequence;
    private Sequence[] allSequences;
    private int sequenceIndex = 0;
    private Executor[] runs;
    private int currentRun = 0;
    private boolean isInterapted = false;
    private boolean isSync;
    private Thread runThread; //makes longer runs without
    private Long startTime;

    public SequenceRunner(Sequence sequence){
        trySetSequence(sequence);
    }

    public SequenceRunner(){

    }

    public SequenceRunner(boolean isSync, Executor... executors){
        this(new Sequence(isSync, executors));
    }

    public SequenceRunner(Sequence... sequences){
        trySetSequence(sequences);
    }

    public SequenceRunner(SequenceInSequence sequence){
        trySetSequence(sequence.GetAllSequences());
    }

    public void update(){
        int countInSec = 3;
        if(runs == null)
            return;

        if(isInterapted) {
            runs[currentRun].stop();
            reset();
            return;
        }
        opMode.telemetry.addData("is finished: ", runs[currentRun].isFinished());

        if(runs[currentRun].isFinished() || !isSync
                || Calendar.getInstance().getTimeInMillis() - startTime >= countInSec*1000){
            currentRun++;
            if(currentRun < runs.length){
                runThread = new Thread(runs[currentRun]);
                runThread.start();
            }
            else{
                sequenceIndex++;
                isFinished = true;
                runs = null;

                if(sequenceIndex < allSequences.length)
                {
                    runningSequence = allSequences[sequenceIndex];
                    runSequence();
                }
                else
                {
                    reset();
                }
            }
        }
    }
    public void runSequence(){
        if(isFinished && runningSequence != null)
        {
            startTime = System.currentTimeMillis();
            isFinished = false;
            runs = runningSequence.getRuns();
            isSync = runningSequence.isSequenceSync();
            currentRun = 0;
            //m_Runs[CurrentRun].run();
            runThread = new Thread(runs[currentRun]);
            runThread.start();
        }
    }

    public void runSequence(Sequence sequence){
        if(isFinished) {
            trySetSequence(sequence);
            runSequence();
        }
    }

    public void runSequence(Sequence[] sequences){
        if(isFinished) {
            trySetSequence(sequences);
            runSequence();
        }
    }

    public void runSequence(SequenceInSequence sequences){
        if(isFinished) {
            trySetSequence(sequences.GetAllSequences());
            runSequence();
        }
    }

    public void trySetSequence(Sequence sequence){
        trySetSequence(new Sequence[] {sequence});
    }

    public void trySetSequence(Sequence[] sequences){
        if(isFinished){
            allSequences = sequences;
            sequenceIndex = 0;
            runningSequence = sequences[sequenceIndex];
            isSync = runningSequence.isSequenceSync();
        }
    }

    private void reset(){
        runs = null;
        sequenceIndex = 0;
        allSequences = null;
        currentRun = 0;
    }
    public boolean IsSequenceDone(){
        return isFinished;
    }

    public void interapt(){
        isInterapted = true;
        update();
    }

}
