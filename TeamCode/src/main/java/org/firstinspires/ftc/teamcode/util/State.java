package org.firstinspires.ftc.teamcode.util;

public interface State {
    /** Called once when the state becomes active. */
    void init();

    /** Called repeatedly while this state is active. */
    void run();

    /** Return true if it’s time to transition to the next state. */
    boolean isDone();
}
