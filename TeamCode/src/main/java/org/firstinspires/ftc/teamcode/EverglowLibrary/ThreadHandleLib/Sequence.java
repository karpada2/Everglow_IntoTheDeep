package org.firstinspires.ftc.teamcode.EverglowLibrary.ThreadHandleLib;

import org.firstinspires.ftc.teamcode.EverglowLibrary.Systems.Executor;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class Sequence {

    private Queue<Executor> runs = new LinkedList<>();
    private Thread thread = new Thread();
    private boolean isRunAsync;
    private ExecutorService service;
    private Queue<Future> futures = new LinkedList<>();


    public Sequence(boolean isRunAsync, Executor... AllRuns) {
        this(isRunAsync, Arrays.asList(AllRuns));
    }

    public Sequence(boolean isRunAsync, List<Executor> AllRuns) {
        runs.addAll(AllRuns);
        this.isRunAsync = isRunAsync;
        if(isRunAsync)
            setExecutorAsync(); // parallel
        else
            setExecutorSync(); //in Queue
    }

    private void setExecutorSync() {
        thread = new Thread(() -> {
            Executor[] arrExe = runs.toArray(new Executor[runs.size()]);

            for (int i = 0; i< runs.size(); i++){
                arrExe[i].run();
                while (!arrExe[i].isFinished() && !Thread.currentThread().isInterrupted()){

                }
                if(Thread.currentThread().isInterrupted())
                    arrExe[i].stop();
            }
        });
    }

    public Executor[] getRuns(){
        return runs.toArray(new Executor[runs.size()]);
    }


    private void setExecutorAsync() {
        if(runs.size() > 0)
            service = Executors.newFixedThreadPool(runs.size());
    }

    public void startSequence() {
        if (isRunAsync) {
            Executor[] arrExe = runs.toArray(new Executor[runs.size()]);
            for (int i = 0; i < runs.size(); i++) {
                //run async the run method in order, save the Future class in Queue of the runs
                futures.add(service.submit(arrExe[i]));
            }
        } else{
            if(!thread.isAlive()) {
                thread.start();
            }
        }
    }

    public boolean isAllDone() {
        if (isRunAsync) {
            for (Future future :
                    futures) {
                if (!future.isDone())
                    return false;
            }
            return true;
        } else
            return isSyncThreadDone();
    }

    public boolean isSyncThreadDone() {
        if (thread != null)
            return !thread.isAlive();
        return true;
    }

    public Queue<Future> getFutures() {
        return futures;
    }

    public Thread getThread() {
        return thread;
    }

    public boolean isSequenceSync() {
        return !isRunAsync;
    }

    public void setRuns(Queue<Executor> runs) {
        //only if there is nothing in there or the size is zero
        if (this.runs != null)
            if (this.runs.size() == 0)
                this.runs = runs;
            else{
                this.runs = runs;
            }

        if(isRunAsync)
            setExecutorAsync();
    }

    public boolean addRun(Executor run) {
        //only add run the thread isn't started yet
        if (isAllDone()) {
            runs.add(run);
            if(isRunAsync)
                setExecutorAsync();
            return true;
        }
        return false;
    }

    public Sequence addSequence(Sequence sequence){
        runs.addAll(sequence.runs);
        return this;
    }

    public boolean isDone(){
        return !thread.isAlive();
    }

    public void interruptSequence(){
        if(isRunAsync)
            service.shutdown();
        else
            thread.interrupt();
    }

    //todo: set an method that alert when sequence done
}
