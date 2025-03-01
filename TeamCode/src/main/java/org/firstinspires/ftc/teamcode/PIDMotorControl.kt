//package org.firstinspires.ftc.teamcode
//
//import com.acmerobotics.dashboard.config.Config
//import com.qualcomm.robotcore.eventloop.opmode.OpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import com.qualcomm.robotcore.hardware.DcMotor
//import kotlin.math.max
//import kotlin.math.min
//
//@TeleOp(name = "PID Motor Control OpMode", group = "TeleOp")
//@Config
//class PIDMotorControl : OpMode() {
//    private var motor: DcMotor? = null
//    private var integral = 0.0
//    private var lastError = 0.0
//    private var lastTime: Long = 0
//
//    override fun init() {
//        // Initialize the motor (configured as "1")
//        motor = hardwareMap.get(DcMotor::class.java, "1")
//        motor!!.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
//        motor!!.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
//        lastTime = System.currentTimeMillis()
//    }
//
//    override fun loop() {
//        // --- Gamepad control for target position adjustment ---
//        // Increase target position by 100 encoder ticks when left bumper is pressed
//        if (gamepad1.left_bumper) {
//            targetPosition += 1
//        }
//        // Decrease target position by 100 encoder ticks when right bumper is pressed
//        if (gamepad1.right_bumper) {
//            targetPosition -= 1
//        }
//
//        // --- PIDF Controller Calculation ---
//        val currentPosition = motor!!.currentPosition
//        val error = targetPosition - currentPosition
//
//        val currentTime = System.currentTimeMillis()
//        var dt = (currentTime - lastTime) / 1000.0
//        if (dt <= 0) {
//            dt = 0.001
//        }
//
//        // Integral accumulation
//        integral += error * dt
//        // Derivative calculation
//        val derivative = (error - lastError) / dt
//        // Feedforward term based on the target (if needed)
//        val feedForward = kF * targetPosition
//
//        // Compute the output power from the PIDF controller
//        var output = (kP * error) + (kI * integral) + (kD * derivative) + feedForward
//
//        // Clip the output to the motor's valid range [-1, 1]
//        output = max(-1.0, min(1.0, output))
//
//        // Apply the computed power to the motor
//        motor!!.power = output
//
//        // Telemetry for monitoring behavior
//        telemetry.addData("Target", targetPosition)
//        telemetry.addData("Current", currentPosition)
//        telemetry.addData("Error", error)
//        telemetry.addData("Output", output)
//        telemetry.update()
//
//        // Prepare for next loop iteration
//        lastError = error.toDouble()
//        lastTime = currentTime
//    }
//
//    companion object {
//        // PIDF constants can be tuned via FTC Dashboard (or similar)
//        @JvmStatic
//        var kP: Double = 0.001 // Proportional coefficient
//        @JvmStatic
//        var kI: Double = 0.0001 // Integral coefficient
//        @JvmStatic
//        var kD: Double = 0.0005 // Derivative coefficient
//        @JvmStatic
//        var kF: Double = 0.0 // Feedforward coefficient
//
//        // Target position can also be adjusted via the dashboard
//        @JvmStatic
//        var targetPosition: Int = 0
//    }
//}