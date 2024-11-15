package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@TeleOp(name = "MainTeleOp", group = "Test")
class MainTeleOp : OpMode() {
    // Declare hardware variables
    private lateinit var motorFrontLeft: DcMotor
    private lateinit var motorBackLeft: DcMotor
    private lateinit var motorFrontRight: DcMotor
    private lateinit var motorBackRight: DcMotor
    private lateinit var imu: BNO055IMU
    private lateinit var armClaw: Servo
    private lateinit var armRotationMotor: DcMotor
    private lateinit var armExtensionMotor: DcMotor
    private lateinit var hangingArm: DcMotorSimple
    private var reachedArmExtensionLimit = false

    // Variables for scaling and gamepad state
    private var speedModes = doubleArrayOf(0.3, 0.55, 0.8, 1.0)
    private var currentSpeedModeIndex = 1 // default to 0.55
    private var prevDpadUp = false
    private var prevDpadDown = false

    // For heading reset
    private var headingOffset = 0.0

    override fun init() {
        // Initialize motors
        motorFrontLeft = hardwareMap.dcMotor["LF"]
        motorBackLeft = hardwareMap.dcMotor["LR"]
        motorFrontRight = hardwareMap.dcMotor["RF"]
        motorBackRight = hardwareMap.dcMotor["RR"]
        armClaw = hardwareMap.servo["claw_grab"]
        armRotationMotor = hardwareMap.dcMotor["arm_rotation"]
        armExtensionMotor = hardwareMap.dcMotor["arm_extension"]
        hangingArm = hardwareMap.dcMotor["hanging"]


        armClaw.scaleRange(-0.1, 0.1)
        // Set all motors to run using encoders
        val motors = arrayOf(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, armRotationMotor, armExtensionMotor)
        for (motor in motors) {
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }

        // Reverse direction of left-side motors
        motorFrontLeft.direction = DcMotorSimple.Direction.REVERSE
        motorBackLeft.direction = DcMotorSimple.Direction.REVERSE

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(BNO055IMU.Parameters())

        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        //ARM ROTATION
        if (gamepad2.dpad_down) {
            armRotationMotor.targetPosition = 1100
            armRotationMotor.power = 0.5
            armRotationMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            //DOWN HAS POWER NEGATIVE
        }
        if (gamepad2.dpad_up) {
            armRotationMotor.targetPosition = 20
            armRotationMotor.power = -0.6
            armRotationMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        // ARM ROTATION WITH JOYSTICK
        //arm rotation joystick
        if (gamepad2.left_stick_y != 0f) {
            armRotationMotor.targetPosition = 1200
            armRotationMotor.power = 0.05 * gamepad2.left_stick_y
            armRotationMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            //
        }
        else{
            armRotationMotor.power = 0.0;
        }
        //ARM EXTENSION
        if (gamepad2.right_stick_y != 0f) {
            armRotationMotor.targetPosition = armExtensionMotor.currentPosition + gamepad2.right_stick_y.toInt() * -5
            armRotationMotor.power = -gamepad1.left_stick_y.toDouble()
            //goes up is ________
            //goes down is ___________
            armRotationMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }


        // CLAW
        if (gamepad2.x) {
            armClaw.setPosition(0.0)
        }
        if (gamepad2.b) {
            armClaw.setPosition(0.3)
        }

        //HANGING
        if (gamepad2.right_bumper){hangingArm.setPower(1.0);}
        else if (gamepad2.left_bumper) {hangingArm.setPower(-1.0);}
        else {hangingArm.setPower(0.0);}





        /////////////////////////////////////////////////////////////////////////////////////////////////
        // Update speed mode based on D-pad input
        val currentDpadUp = gamepad1.dpad_up
        val currentDpadDown = gamepad1.dpad_down

        if (currentDpadUp && !prevDpadUp) {
            // D-pad up was just pressed
            if (currentSpeedModeIndex < speedModes.size - 1) {
                currentSpeedModeIndex++
            }
        }

        if (currentDpadDown && !prevDpadDown) {
            // D-pad down was just pressed
            if (currentSpeedModeIndex > 0) {
                currentSpeedModeIndex--
            }
        }

        // Update previous button states
        prevDpadUp = currentDpadUp
        prevDpadDown = currentDpadDown

        val speedScale = speedModes[currentSpeedModeIndex]

        // Read joystick values
        val y = -gamepad1.left_stick_y.toDouble() // Forward/backward
        val x = gamepad1.left_stick_x.toDouble() * 1.1 // Left/right strafe
        val rx = gamepad1.right_stick_x.toDouble() // Rotation

        // Reset heading when 'A' button is pressed
        if (gamepad1.a) {
            headingOffset = imu.angularOrientation.firstAngle.toDouble()
        }

        // Get robot heading from IMU and apply offset
        val botHeadingRadians = -Math.toRadians(imu.angularOrientation.firstAngle.toDouble() - headingOffset)

        // Rotate the field-centric inputs based on the robot's heading
        val rotX = x * cos(botHeadingRadians) - y * sin(botHeadingRadians)
        val rotY = x * sin(botHeadingRadians) + y * cos(botHeadingRadians)

        // Compute motor power values
        val denominator = max(abs(rotY) + abs(rotX) + abs(rx), 1.0)
        var frontLeftPower = (rotY + rotX + rx) / denominator
        var backLeftPower = (rotY - rotX + rx) / denominator
        var frontRightPower = (rotY - rotX - rx) / denominator
        var backRightPower = (rotY + rotX - rx) / denominator

        // Apply speed scaling, with turbo mode when right bumper is pressed
        val isTurboMode = gamepad1.right_bumper
        val adjustedSpeedScale = if (isTurboMode) 1.0 else speedScale

        frontLeftPower *= adjustedSpeedScale
        backLeftPower *= adjustedSpeedScale
        frontRightPower *= adjustedSpeedScale
        backRightPower *= adjustedSpeedScale

        // Set motor powers
        motorFrontLeft.power = frontLeftPower
        motorBackLeft.power = backLeftPower
        motorFrontRight.power = frontRightPower
        motorBackRight.power = backRightPower

        // Send telemetry data to driver station
        telemetry.addData("Front Left Power", frontLeftPower)
        telemetry.addData("Back Left Power", backLeftPower)
        telemetry.addData("Front Right Power", frontRightPower)
        telemetry.addData("Back Right Power", backRightPower)
        telemetry.addData("Speed Mode", speedScale)
        telemetry.addData("Turbo Mode", if (isTurboMode) "ON" else "OFF")
        telemetry.addData("Heading", Math.toDegrees(botHeadingRadians))
        telemetry.update()

    }
}