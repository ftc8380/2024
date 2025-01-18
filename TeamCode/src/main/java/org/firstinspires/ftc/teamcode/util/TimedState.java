package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class TimedState implements State {
    private final OpMode opMode;
    private final double durationSecs;
    private double startTime;

    public TimedState(OpMode opMode, double durationSecs) {
        this.opMode = opMode;
        this.durationSecs = durationSecs;
    }

    @Override
    public void init() {
        startTime = opMode.getRuntime();
        onStart();
    }

    /** Optional “do something once” at the start. */
    public void onStart() { }

    @Override
    public void run() {
        // Usually empty, unless you want something that runs continuously
    }

    @Override
    public boolean isDone() {
        return (opMode.getRuntime() - startTime) >= durationSecs;
    }
}