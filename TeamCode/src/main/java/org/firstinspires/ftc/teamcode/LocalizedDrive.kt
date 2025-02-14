package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import kotlin.math.cos
import kotlin.math.sin

/**
 * TeleOp that uses Roadrunner’s mecanum drive for localization.
 *
 * Controls:
 * - Right Trigger: Adds forward (field-oriented) motion.
 * - Left Trigger: Adds backward (field-oriented) motion.
 * - Left Stick: Controls additional translation (field oriented).
 * - Right Stick (X axis): Controls turning.
 *
 * In init (init_loop) you can adjust the initial pose estimate:
 * - Gamepad1 A/B: Increase/Decrease X.
 * - Gamepad1 Y/X: Increase/Decrease Y.
 * - Gamepad1 Right/Left Bumper: Increase/Decrease heading (in 5° increments).
 */
@TeleOp(name = "Localized Drive TeleOp", group = "drive")
class LocalizedDriveTeleOp : OpMode() {
    private var drive: MecanumDrive? = null

    // Initial pose estimate (x, y in inches and heading in radians)
    private var initialPose = Pose2d(0.0, 0.0, 0.0)

    override fun init() {
        // Initialize the drive. Pass in the hardwareMap.
        drive = MecanumDrive(hardwareMap)

        // Set the initial pose estimate.
        drive!!.poseEstimate = initialPose

        // In init, tell the user how to adjust the initial pose.
        telemetry.addLine("Adjust initial pose using gamepad:")
        telemetry.addLine("  A/B: Increase/Decrease X")
        telemetry.addLine("  Y/X: Increase/Decrease Y")
        telemetry.addLine("  Right/Left Bumper: Increase/Decrease Heading (5° steps)")
        telemetry.addLine("Press START when ready.")
        telemetry.update()
    }

    override fun init_loop() {
        // Allow the user to adjust the initial pose estimate.
        // Increase X with A, decrease with B.
        if (gamepad1.a) {
            initialPose = Pose2d(initialPose.x + 0.5, initialPose.y, initialPose.heading)
            sleep(100) // small delay to prevent rapid-fire changes
        }
        if (gamepad1.b) {
            initialPose = Pose2d(initialPose.x - 0.5, initialPose.y, initialPose.heading)
            sleep(100)
        }
        // Increase Y with Y, decrease with X.
        if (gamepad1.y) {
            initialPose = Pose2d(initialPose.x, initialPose.y + 0.5, initialPose.heading)
            sleep(100)
        }
        if (gamepad1.x) {
            initialPose = Pose2d(initialPose.x, initialPose.y - 0.5, initialPose.heading)
            sleep(100)
        }
        // Adjust heading: bumpers add/subtract 5° (converted to radians).
        val angleIncrement = Math.toRadians(5.0)
        if (gamepad1.right_bumper) {
            initialPose = Pose2d(initialPose.x, initialPose.y, initialPose.heading + angleIncrement)
            sleep(100)
        }
        if (gamepad1.left_bumper) {
            initialPose = Pose2d(initialPose.x, initialPose.y, initialPose.heading - angleIncrement)
            sleep(100)
        }

        // Update the drive's pose estimate.
        drive!!.poseEstimate = initialPose

        // Display the current initial pose.
        telemetry.addData("Initial Pose", initialPose.toString())
        telemetry.update()
    }

    override fun loop() {
        // Update localization
        drive!!.update()

        // Read gamepad inputs (field-oriented)
        val forward = -gamepad1.left_stick_y.toDouble() // forward/backward
        val strafe = -gamepad1.left_stick_x.toDouble() // left/right
        val turn = -gamepad1.right_stick_x.toDouble() // rotation

        // Add trigger-based movement for more controlled speed adjustment
        val forward_adjustment = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()

        // Combine the field-oriented movement
        val adjustedForward = forward + forward_adjustment

        // Directly pass the weighted drive power
        drive!!.setWeightedDrivePower(Pose2d(adjustedForward, strafe, turn))

        // Telemetry for debugging
        val currentPose = drive!!.poseEstimate
        telemetry.addData("Pose", currentPose)
        telemetry.addData("X", currentPose.x)
        telemetry.addData("Y", currentPose.y)
        telemetry.addData("Heading", Math.toDegrees(currentPose.heading))
        telemetry.update()
    }

    /**
     * Helper method: Sleep for a given number of milliseconds.
     * (Note: In an iterative OpMode, this is not ideal. In a real implementation,
     * consider using a timer or state-machine logic to debounce inputs.)
     */
    private fun sleep(millis: Long) {
        try {
            Thread.sleep(millis)
        } catch (e: InterruptedException) {
            Thread.currentThread().interrupt()
        }
    }
}

/**
 * Extension function for Pose2d that rotates the x and y components by the given angle (in radians).
 * The heading remains unchanged.
 */
fun Pose2d.rotated(angle: Double): Pose2d {
    val cos = cos(angle)
    val sin = sin(angle)
    return Pose2d(
        x * cos - y * sin,
        x * sin + y * cos,
        heading
    )
}