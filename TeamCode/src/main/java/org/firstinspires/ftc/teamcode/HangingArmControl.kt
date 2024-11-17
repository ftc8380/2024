package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "HangingArmControl", group = "Test")
class HangingArmControl : OpMode() {
    // Declare hardware variables
    private lateinit var hangingArm: DcMotor

    override fun init() {
        // Initialize the hanging arm motor
        hangingArm = hardwareMap.dcMotor["hanging"]

        // Set motor to use direct power control
        hangingArm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        // Set motor to brake when no power is applied
        hangingArm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Control hanging arm power with Gamepad 2
        when {
            gamepad2.dpad_up -> hangingArm.power = 1.0  // Move arm up
            gamepad2.dpad_down -> hangingArm.power = -1.0 // Move arm down
            else -> hangingArm.power = 0.0 // Stop arm movement
        }

        // Send telemetry data to the driver station
        telemetry.addData("Hanging Arm Power", hangingArm.power)
        telemetry.addData("Hanging Arm Position", hangingArm.currentPosition)
        telemetry.update()
    }
}