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
*    | ( x direction)
*    |
*    v
*    <----( y direction )---->

*        (forward)
*    /--------------\
*    |     ____     |
*    |     ----     |    <- Perpendicular Wheel
*    |           || |
*    |           || |    <- Parallel Wheel
*    |              |
*    |              |
*    \--------------/
*
*/
class TwoWheelTrackingLocalizer(hardwareMap: HardwareMap, private val drive: MecanumDrive) :
    TwoTrackingWheelLocalizer(
        Arrays.asList(
            Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
            Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0))
        )
    ) {
    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private val parallelEncoder = Encoder(
        hardwareMap.get(
            DcMotorEx::class.java, "hanging"
        )
    )
    private val perpendicularEncoder = Encoder(
        hardwareMap.get(
            DcMotorEx::class.java, "perpendicularEncoder"
        )
    )

    init {
        parallelEncoder.direction = Encoder.Direction.REVERSE
        perpendicularEncoder.direction = Encoder.Direction.REVERSE

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    override fun getHeading(): Double {
        return drive.rawExternalHeading
    }

    override fun getHeadingVelocity(): Double? {
        return drive.getExternalHeadingVelocity()
    }

    override fun getWheelPositions(): List<Double> {
        return Arrays.asList(
            encoderTicksToInches(parallelEncoder.currentPosition.toDouble()),
            encoderTicksToInches(perpendicularEncoder.currentPosition.toDouble())
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
            encoderTicksToInches(parallelEncoder.rawVelocity),
            encoderTicksToInches(perpendicularEncoder.rawVelocity)
        )
    }

    companion object {
        private var TICKS_PER_REV: Double = 2000.0
        private var WHEEL_RADIUS: Double = 1.25 // in
        private var GEAR_RATIO: Double = 1.0 // output (wheel) speed / input (encoder) speed

        var PARALLEL_X: Double = 5.5 // X is the up and down direction
        var PARALLEL_Y: Double = 2.5 // Y is the strafe direction

        var PERPENDICULAR_X: Double = 5.5
        var PERPENDICULAR_Y: Double = 3.0

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }
}
