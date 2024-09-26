package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.*
import org.opencv.core.*
import org.opencv.imgproc.Imgproc

@TeleOp(name = "ObjectDetectionMotorControl", group = "Test")
class TestOp : OpMode() {
    // Declare the motor and webcam
    private lateinit var motor: DcMotor
    private lateinit var webcam: OpenCvCamera

    // FTC Dashboard instance
    private lateinit var dashboard: FtcDashboard

    override fun init() {
        // Initialize the motor
        motor = hardwareMap.get(DcMotor::class.java, "motor")

        // Initialize the webcam
        val webcamName = hardwareMap.get(WebcamName::class.java, "webcam")
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.packageName
        )
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        // Open the webcam and set it to stream to the FTC Dashboard
        webcam.setPipeline(ObjectDetectionPipeline())
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
                FtcDashboard.getInstance().startCameraStream(webcam, 0.0)
            }

            override fun onError(errorCode: Int) {
                telemetry.addData("Error", "Webcam could not open")
            }
        })

        // Initialize dashboard and telemetry
        dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)

        // Inform the driver that the initialization is complete
        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        // Use the detected object size from the pipeline (updated in real-time by the pipeline)
        val objectSize = ObjectDetectionPipeline.detectedObjectSize

        // Map the object size to motor power (0 to 0.5)
        val motorPower = ((10* objectSize) / (320.0 * 240.0)) * 0.5

        // Set motor power
        motor.power = motorPower

        // Display telemetry to both the driver station and FTC Dashboard
        telemetry.addData("Motor Power", motorPower)
        telemetry.addData("Detected Object Size", objectSize)
        telemetry.update()

        // Send motor power and detected object size data to the dashboard
        val packet = TelemetryPacket()
        packet.put("Motor Power", motorPower)
        packet.put("Detected Object Size", objectSize)
        dashboard.sendTelemetryPacket(packet)
    }

    override fun stop() {
        // Stop the motor when the OpMode is stopped
        motor.power = 0.0

        // Stop the camera stream
        webcam.stopStreaming()
    }

    // Pipeline class for rudimentary object detection based on color thresholding
    class ObjectDetectionPipeline : OpenCvPipeline() {
        companion object {
            var detectedObjectSize: Double = 0.0
        }

        // Define color range for detection (in HSV format, e.g., for detecting red objects)
        private val lowerBound = Scalar(0.0, 100.0, 100.0) // Adjust values as needed
        private val upperBound = Scalar(10.0, 255.0, 255.0)

        private val hsvMat = Mat()
        private val mask = Mat()
        private val contours = ArrayList<MatOfPoint>()

        override fun processFrame(input: Mat): Mat {
            // Convert the input frame to HSV
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV)

            // Threshold the image to get only the color we want
            Core.inRange(hsvMat, lowerBound, upperBound, mask)

            // Find contours in the mask
            contours.clear()
            val hierarchy = Mat()
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)

            // Find the largest contour by area (this is the detected object)
            var maxArea = 0.0
            for (contour in contours) {
                val area = Imgproc.contourArea(contour)
                if (area > maxArea) {
                    maxArea = area
                }
            }

            // Update the detected object size
            detectedObjectSize = maxArea

            // Draw the largest contour (if any) on the input frame for visualization
            if (contours.isNotEmpty()) {
                Imgproc.drawContours(input, contours, -1, Scalar(0.0, 255.0, 0.0), 2)
            }

            // Return the modified frame (with contours) for display
            return input
        }
    }
}