package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class PIDMotorControl extends OpMode {

    private DcMotor motor;
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime;

    @Override
    public void init() {
        // Initialize the motor (configured as "1")
        motor = hardwareMap.get(DcMotor.class, "1");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        // --- Gamepad control for target position adjustment ---
        // Increase target position by 1 tick when left bumper is pressed
        if (gamepad1.left_bumper) {
            targetPosition += 1;
        }
        // Decrease target position by 1 tick when right bumper is pressed
        if (gamepad1.right_bumper) {
            targetPosition -= 1;
        }

        // --- PIDF Controller Calculation ---
        int currentPosition = motor.getCurrentPosition();
        int error = targetPosition - currentPosition;

        long currentTime = System.currentTimeMillis();
        double dt = (currentTime - lastTime) / 1000.0;
        if (dt <= 0) {
            dt = 0.001;
        }

        // Integral accumulation
        integral += error * dt;
        // Derivative calculation
        double derivative = (error - lastError) / dt;
        // Feedforward term based on the target (if needed)
        double feedForward = kF * targetPosition;

        // Compute the output power from the PIDF controller
        double output = (kP * error) + (kI * integral) + (kD * derivative) + feedForward;
        // Clip the output to the motor's valid range [-1, 1]
        output = Math.max(-1.0, Math.min(1.0, output));

        // Apply the computed power to the motor
        motor.setPower(output);

        // Telemetry for monitoring behavior
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Current", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Output", output);
        telemetry.update();

        // Prepare for next loop iteration
        lastError = error;
        lastTime = currentTime;
    }

    // PIDF constants and target position accessible via dashboard
    public static double kP = 0.001;   // Proportional coefficient
    public static double kI = 0.0001;  // Integral coefficient
    public static double kD = 0.0005;  // Derivative coefficient
    public static double kF = 0.0;     // Feedforward coefficient
    public static int targetPosition = 0;
}
