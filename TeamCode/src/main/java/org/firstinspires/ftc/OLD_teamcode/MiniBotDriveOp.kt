package org.firstinspires.ftc.robotcontroller.internal

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@TeleOp(name = "EnhancedMiniBotDriveOp", group = "Test")
class EnhancedMiniBotDriveOp : OpMode() {
    // Declare hardware variables
    private var motorFrontLeft: DcMotor? = null
    private var motorBackLeft: DcMotor? = null
    private var motorFrontRight: DcMotor? = null
    private var motorBackRight: DcMotor? = null
    private var imu: BNO055IMU? = null
    private lateinit var camera: OpenCvCamera

    // Webcam feed
    private val dashboard = FtcDashboard.getInstance()

    // Variables for scaling and gamepad state
    private var scale = 0.55
    private var slowMode = false

    override fun init() {
        // Initialize motors
        motorFrontLeft = hardwareMap.dcMotor["LF"]
        motorBackLeft = hardwareMap.dcMotor["LR"]
        motorFrontRight = hardwareMap.dcMotor["RF"]
        motorBackRight = hardwareMap.dcMotor["RR"]

        // Enable encoder mode
        motorFrontLeft!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorBackLeft!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorFrontRight!!.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motorBackRight!!.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // Reverse direction of left-side motors
        motorFrontLeft!!.direction = DcMotorSimple.Direction.REVERSE
        motorBackLeft!!.direction = DcMotorSimple.Direction.REVERSE

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu!!.initialize(BNO055IMU.Parameters())

        // Initialize the camera and stream to dashboard
        val webcamName = hardwareMap.get(WebcamName::class.java, "webcam")
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

// Set MJPEG for better performance
        camera.setPipeline(object : OpenCvPipeline() {
            override fun processFrame(input: Mat): Mat {
                // Convert the image to grayscale
                val gray = Mat()
                Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY)

                // Blur the image to reduce noise
                val blurred = Mat()
                Imgproc.GaussianBlur(gray, blurred, Size(5.0, 5.0), 0.0)

                // Detect edges using Canny
                val edges = Mat()
                Imgproc.Canny(blurred, edges, 100.0, 200.0)

                // Find contours
                val contours = mutableListOf<MatOfPoint>()
                val hierarchy = Mat()
                Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)

                // Draw the contours in green
                for (contour in contours) {
                    Imgproc.drawContours(input, listOf(contour), -1, Scalar(0.0, 255.0, 0.0), 2)
                }

                // Return the image with contours drawn
                return input
            }
        })

        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                // Start streaming with MJPEG format and resolution 720x1280
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance().startCameraStream(camera, 30.0)
            }

            override fun onError(errorCode: Int) {
                telemetry.addData("Error", "Webcam could not open")
            }
        })
        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Slow mode scaling based on D-Pad input
        slowMode = gamepad1.dpad_down

        // Read joystick values
        val y = -gamepad1.left_stick_y.toDouble() // Forward/backward
        val x = gamepad1.left_stick_x * 1.1 // Left/right strafe
        val rx = gamepad1.right_stick_x.toDouble() // Rotation

        // Get robot heading from IMU
        val botHeading = -imu!!.angularOrientation.firstAngle.toDouble()

        // Rotate the field-centric inputs based on the robot's heading
        val rotX = x * cos(botHeading) - y * sin(botHeading)
        val rotY = x * sin(botHeading) + y * cos(botHeading)

        // Compute motor power values
        val denominator = max(abs(y) + abs(x) + abs(rx), 1.0)
        val frontLeftPower = scale * (rotY + rotX + rx) / denominator
        val backLeftPower = scale * (rotY - rotX + rx) / denominator
        val frontRightPower = scale * (rotY - rotX - rx) / denominator
        val backRightPower = scale * (rotY + rotX - rx) / denominator

        // Adjust power scaling in slow mode
        val finalScale = if (slowMode) 0.4 else 0.55

        // Set motor powers
        motorFrontLeft!!.power = frontLeftPower * finalScale
        motorBackLeft!!.power = backLeftPower * finalScale
        motorFrontRight!!.power = frontRightPower * finalScale
        motorBackRight!!.power = backRightPower * finalScale

        // Send telemetry data to driver station
        telemetry.addData("Front Left Power", frontLeftPower)
        telemetry.addData("Back Left Power", backLeftPower)
        telemetry.addData("Front Right Power", frontRightPower)
        telemetry.addData("Back Right Power", backRightPower)
        telemetry.update()
    }
}