package org.firstinspires.ftc.teamcode.drive.opmode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.drive.MecanumDrive
import org.firstinspires.ftc.teamcode.util.Encoder
import java.util.Arrays

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | (x direction)
 *    |
 *    v
 *    <---- (y direction) ---->
 *
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular (lateral) Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 * In this implementation the lateral (perpendicular) wheel is mounted at an X offset
 * (PERPENDICULAR_X) from the robot’s center. When the robot rotates, that wheel
 * travels an extra distance = PERPENDICULAR_X * (change in heading).
 * We compensate for this by subtracting that term from the lateral encoder reading.
 */
class TwoWheelTrackingLocalizer(
    hardwareMap: HardwareMap,
    private val drive: MecanumDrive
) : TwoTrackingWheelLocalizer(
    // The poses here are used for kinematics. They should reflect the physical mounting.
    // Note: The PERPENDICULAR wheel’s pose remains unchanged.
    Arrays.asList(
        Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
        Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
    )
) {
    // Parallel (forward) encoder
    private val parallelEncoder = Encoder(
        hardwareMap.get(DcMotorEx::class.java, "hanging")
    )
    // Perpendicular (lateral) encoder
    private val perpendicularEncoder = Encoder(
        hardwareMap.get(DcMotorEx::class.java, "perpendicularEncoder")
    )

    // Store the heading when the localizer is created. This lets us subtract off the
    // rotational contribution relative to the start.
    private val headingOffset: Double = getHeading()

    init {
        parallelEncoder.direction = Encoder.Direction.REVERSE
//        perpendicularEncoder.direction = Encoder.Direction.REVERSE

        // TODO: Reverse any encoders as necessary using Encoder.setDirection(...)
    }

    override fun getHeading(): Double {
        return drive.rawExternalHeading
    }

    override fun getHeadingVelocity(): Double? {
        return drive.getExternalHeadingVelocity()
    }

    override fun getWheelPositions(): List<Double> {
        // Get the raw distances (in inches) from the encoders.
        val parallelPos = encoderTicksToInches(parallelEncoder.currentPosition.toDouble())
        val rawPerpPos = encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble())

        // Compensate for the rotational component.
        // The perpendicular (lateral) wheel is mounted PERPENDICULAR_X inches forward,
        // so any change in heading will cause an extra lateral displacement of:
        //   PERPENDICULAR_X * (currentHeading - headingOffset)
        // We subtract that to isolate the pure translation.
        val correctedPerpPos = rawPerpPos - PERPENDICULAR_X * (getHeading() - headingOffset)

        return Arrays.asList(
            parallelPos,
            correctedPerpPos
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return Arrays.asList(
            encoderTicksToInches(parallelEncoder.rawVelocity),
            encoderTicksToInches(perpendicularEncoder.rawVelocity)
        )
    }

    companion object {
        // Encoder and wheel parameters
        private var TICKS_PER_REV: Double = 2000.0
        private var WHEEL_RADIUS: Double = 0.63 // in inches
        private var GEAR_RATIO: Double = 1.0 // output (wheel) speed / input (encoder) speed

        // Pose of the tracking wheels relative to the robot's center.
        var PARALLEL_X: Double = 5.5 // forward offset for parallel wheel (in inches)
        var PARALLEL_Y: Double = 2.5 // lateral offset for parallel wheel (in inches)

        var PERPENDICULAR_X: Double = 5.5 // forward offset for perpendicular (lateral) wheel (in inches)
        var PERPENDICULAR_Y: Double = 3.0 // lateral offset for perpendicular (lateral) wheel (in inches)

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}