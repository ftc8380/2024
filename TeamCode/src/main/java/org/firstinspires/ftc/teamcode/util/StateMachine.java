package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.ArrayList;
import java.util.List;

public class StateMachine {
    private final List<State> states = new ArrayList<>();
    private int currentIndex = 0;
    private boolean initialized = false;
    private boolean waiting = false;       // Indicates if we're waiting for user input
    private Integer pendingIndex = null;   // Holds requested state index when waiting

    public void addState(State state) {
        states.add(state);
    }

    public boolean isFinished() {
        return currentIndex >= states.size();
    }

    /**
     * Allows external code (or gamepad input) to request a state change.
     * The request will be honored when the machine is in a waiting state.
     * @param index The index of the desired state to transition to.
     */
    public void requestStateChange(int index) {
        if (index >= 0 && index < states.size()) {
            pendingIndex = index;
        }
    }

    /**
     * Update loop that uses an OpMode to read gamepad inputs for manual control,
     * and waits for user input before starting a new state after completion of the current one.
     */
    public void update(OpMode opmode) {
        if (isFinished()) return;

        // Check gamepad input to request a state change.
        if (opmode.gamepad1.dpad_left) {
            int desiredIndex = Math.max(0, currentIndex - 1);
            requestStateChange(desiredIndex);
        } else if (opmode.gamepad1.dpad_right) {
            int desiredIndex = Math.min(states.size() - 1, currentIndex + 1);
            requestStateChange(desiredIndex);
        }

        // If the machine is waiting for user input to transition to a new state.
        if (waiting) {
            if (pendingIndex != null) {
                // Transition to the requested state.
                currentIndex = pendingIndex;
                pendingIndex = null;
                waiting = false;
                initialized = false;
            } else {
                // No new state requested yet; wait here.
                return;
            }
        }

        // Initialize current state if not yet initialized.
        if (!initialized) {
            states.get(currentIndex).init();
            initialized = true;
        }

        // Run the current state.
        State current = states.get(currentIndex);
        current.run();

        // If the current state is done, set waiting to true and pause transition.
        if (current.isDone()) {
            waiting = true;
        }
    }

    /**
     * Fallback update method without OpMode/gamepad control.
     */
    public void update() {
        if (isFinished()) return;

        if (!initialized) {
            states.get(currentIndex).init();
            initialized = true;
        }

        State current = states.get(currentIndex);
        current.run();

        if (current.isDone()) {
            currentIndex++;
            initialized = false;
        }
    }
}