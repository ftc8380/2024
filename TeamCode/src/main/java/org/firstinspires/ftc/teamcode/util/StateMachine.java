package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.List;

public class StateMachine {
    private final List<State> states = new ArrayList<>();
    private int currentIndex = 0;
    private boolean initialized = false;

    public void addState(State state) {
        states.add(state);
    }

    public boolean isFinished() {
        return currentIndex >= states.size();
    }

    public void update() {
        if (isFinished()) return;

        // Initialize state if first time
        if (!initialized) {
            states.get(currentIndex).init();
            initialized = true;
        }

        // Run the current state
        State current = states.get(currentIndex);
        current.run();

        // Check if done
        if (current.isDone()) {
            currentIndex++;
            initialized = false;  // so next state’s init() will run
        }
    }
}