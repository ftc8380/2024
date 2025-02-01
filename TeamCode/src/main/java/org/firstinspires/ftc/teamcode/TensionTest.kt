package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "Tension Servo Control", group = "Example")
class TensionServoControl : OpMode() {
    // Declare the servo
    private var tensionServo: Servo? = null

    // Servo position variables
    private var servoPosition = 0.5 // Initialize to mid-position

    // Variables for debounce logic
    private var previousAState = false
    private var previousBState = false

    override fun init() {
        // Initialize the servo from the hardware map
        tensionServo = hardwareMap.get(Servo::class.java, "tensionServo")


        // Set initial servo position
        tensionServo!!.position = servoPosition


        // Provide initial telemetry
        telemetry.addData("Status", "Initialized")
        telemetry.addData("Servo Position", servoPosition)
        telemetry.update()
    }

    override fun loop() {
        // Handle Gamepad1 A button for increasing servo position
        if (gamepad1.a && !previousAState) {
            // Button was just pressed
            servoPosition += POSITION_INCREMENT
            // Clamp the position to the allowed range
            servoPosition = clamp(servoPosition, MIN_POSITION, MAX_POSITION)
            // Update the servo position
            tensionServo!!.position = servoPosition


            // Provide feedback
            telemetry.addData("Action", "A pressed: Increased position")
        }
        // Update previous A state
        previousAState = gamepad1.a

        // Handle Gamepad1 B button for decreasing servo position
        if (gamepad1.b && !previousBState) {
            // Button was just pressed
            servoPosition -= POSITION_INCREMENT
            // Clamp the position to the allowed range
            servoPosition = clamp(servoPosition, MIN_POSITION, MAX_POSITION)
            // Update the servo position
            tensionServo!!.position = servoPosition


            // Provide feedback
            telemetry.addData("Action", "B pressed: Decreased position")
        }
        // Update previous B state
        previousBState = gamepad1.b

        // Update telemetry with current servo position
        telemetry.addData("Servo Position", String.format("%.2f", servoPosition))
        telemetry.addData("Press A to Increase", "")
        telemetry.addData("Press B to Decrease", "")
        telemetry.update()
    }

    /**
     * Clamps a value between a minimum and maximum value.
     *
     * @param value The value to clamp.
     * @param min   The minimum allowable value.
     * @param max   The maximum allowable value.
     * @return The clamped value.
     */
    private fun clamp(value: Double, min: Double, max: Double): Double {
        return max(min, min(max, value))
    }

    companion object {
        private const val POSITION_INCREMENT = 0.05
        private const val MIN_POSITION = 0.0
        private const val MAX_POSITION = 1.0
    }
}