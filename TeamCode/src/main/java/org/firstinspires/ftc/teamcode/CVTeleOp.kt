package org.firstinspires.ftc.teamcode

import android.graphics.Bitmap
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.android.Utils
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.*
import java.util.*

@TeleOp(name = "DistanceEstimationOpMode", group = "Iterative Opmode")
class DistanceEstimationOpMode : OpMode() {

    private lateinit var webcam: OpenCvWebcam
    private lateinit var dashboard: FtcDashboard
    private lateinit var pipeline: DistanceEstimationPipeline

    override fun init() {
        // Initialize the webcam and dashboard
        val webcamName = hardwareMap.get(WebcamName::class.java, "webcam")
        dashboard = FtcDashboard.getInstance()
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        // Set the custom pipeline
        pipeline = DistanceEstimationPipeline(telemetry, dashboard)
        webcam.setPipeline(pipeline)

        // Set resolution
        val FRAME_WIDTH = 1280
        val FRAME_HEIGHT = 720

        // Open camera device asynchronously
        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                telemetry.addData("Error", "Could not open camera")
                telemetry.update()
            }
        })
    }

    override fun loop() {
        // Update telemetry with the smoothed distance
        telemetry.addData("Distance", "%.2f inches", pipeline.getSmoothedDistance())
        telemetry.update()

        // Send telemetry to the dashboard
        val packet = TelemetryPacket()
        packet.put("Distance", pipeline.getSmoothedDistance())
        dashboard.sendTelemetryPacket(packet)
    }

    override fun stop() {
        // Stop camera streaming
        webcam.stopStreaming()
    }
}

class DistanceEstimationPipeline(
    private val telemetry: Telemetry,
    private val dashboard: FtcDashboard
) : OpenCvPipeline() {

    // Known width of the cuboid in millimeters
    private val KNOWN_WIDTH = 86.0 // mm

    // Camera calibration parameters
    private val cameraMatrix = Mat(3, 3, CvType.CV_64FC1)
    private val distCoeffs = Mat(1, 5, CvType.CV_64FC1)

    // New camera matrix and undistortion maps
    private val newCameraMatrix = Mat()
    private val map1 = Mat()
    private val map2 = Mat()

    // Focal length of the camera (in pixels)
    private var FOCAL_LENGTH = 0.0

    // Scaling factor based on observed measurements
    private val SCALING_FACTOR = 22.0 / 90.0

    // Variables for smoothing
    private val positions: Deque<Double> = ArrayDeque(5)
    private var lastDetection: Double? = null

    // Smoothed distance value
    private var smoothedDistance = 0.0

    // Mats declared as member variables
    private val undistorted = Mat()
    private val hsv = Mat()
    private val mask1 = Mat()
    private val mask2 = Mat()
    private val mask = Mat()
    private val maskClean = Mat()
    private val hierarchy = Mat()
    private val kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, Size(5.0, 5.0))
    private val contour2f = MatOfPoint2f()
    private val boxPoints = MatOfPoint2f()

    // Flag to check if undistortion maps are initialized
    private var isInitialized = false

    init {
        // Initialize camera matrix
        cameraMatrix.put(0, 0, 3.03224995e+03)
        cameraMatrix.put(0, 1, 0.0)
        cameraMatrix.put(0, 2, 1.16841350e+03)
        cameraMatrix.put(1, 0, 0.0)
        cameraMatrix.put(1, 1, 3.05357098e+03)
        cameraMatrix.put(1, 2, 6.28322282e+02)
        cameraMatrix.put(2, 0, 0.0)
        cameraMatrix.put(2, 1, 0.0)
        cameraMatrix.put(2, 2, 1.0)

        // Initialize distortion coefficients
        distCoeffs.put(0, 0, 3.14152199e-01)
        distCoeffs.put(0, 1, -6.31960706e+00)
        distCoeffs.put(0, 2, -2.62694865e-02)
        distCoeffs.put(0, 3, 1.82763062e-03)
        distCoeffs.put(0, 4, 6.68522073e+01)

        // Set the focal length from the camera matrix
        FOCAL_LENGTH = cameraMatrix.get(0, 0)[0]
    }

    fun getSmoothedDistance(): Double {
        return smoothedDistance
    }

    override fun processFrame(input: Mat): Mat {
        if (!isInitialized) {
            // Compute the new optimal camera matrix
            val imageSize = input.size()
            Calib3d.getOptimalNewCameraMatrix(
                cameraMatrix,
                distCoeffs,
                imageSize,
                1.0,  // Alpha parameter, 1.0 retains all pixels
            )

            // Compute undistortion and rectification maps
            Calib3d.initUndistortRectifyMap(
                cameraMatrix,
                distCoeffs,
                Mat.eye(3, 3, CvType.CV_64FC1),  // Identity matrix for rectification
                newCameraMatrix,
                imageSize,
                CvType.CV_32FC1,
                map1,
                map2
            )

            isInitialized = true
        }

        // Undistort the frame using the precomputed maps
        Imgproc.remap(input, undistorted, map1, map2, Imgproc.INTER_LINEAR)

        // Convert to HSV color space
        Imgproc.cvtColor(undistorted, hsv, Imgproc.COLOR_RGB2HSV)

        // Define red color ranges
        val lowerRed1 = Scalar(0.0, 100.0, 100.0)
        val upperRed1 = Scalar(10.0, 255.0, 255.0)
        val lowerRed2 = Scalar(160.0, 100.0, 100.0)
        val upperRed2 = Scalar(179.0, 255.0, 255.0)

        // Create masks
        Core.inRange(hsv, lowerRed1, upperRed1, mask1)
        Core.inRange(hsv, lowerRed2, upperRed2, mask2)
        Core.bitwise_or(mask1, mask2, mask)

        // Morphological operations
        Imgproc.morphologyEx(mask, maskClean, Imgproc.MORPH_OPEN, kernel)
        Imgproc.dilate(maskClean, maskClean, kernel)

        // Find contours
        val contours = ArrayList<MatOfPoint>()
        Imgproc.findContours(maskClean, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)

        if (contours.isNotEmpty()) {
            // Find the largest contour
            val largestContour = contours.maxByOrNull { Imgproc.contourArea(it) }
            if (largestContour != null && Imgproc.contourArea(largestContour) > 1000) {
                // Convert contour to MatOfPoint2f
                largestContour.convertTo(contour2f, CvType.CV_32F)

                // Minimum area rectangle
                val rect = Imgproc.minAreaRect(contour2f)

                // Get box points
                Imgproc.boxPoints(rect, boxPoints)

                // Convert boxPoints to MatOfPoint (integer coordinates)
                val boxPointsInt = MatOfPoint()
                boxPoints.convertTo(boxPointsInt, CvType.CV_32S)

                // Get points array
                val points = boxPointsInt.toArray()

                // Ensure that we have exactly 4 points
                if (points.size == 4) {
                    Imgproc.polylines(
                        undistorted,
                        listOf(MatOfPoint(*points)),
                        true, // Closed contour
                        Scalar(0.0, 255.0, 0.0),
                        2
                    )
                }

                // Calculate distance
                val perceivedWidth = minOf(rect.size.width, rect.size.height)
                val distanceMm = (KNOWN_WIDTH * FOCAL_LENGTH) / perceivedWidth
                val distanceIn = distanceMm / 25.4
                val distanceAdjusted = distanceIn * SCALING_FACTOR

                // Smoothing
                if (positions.size >= 5) {
                    positions.removeFirst()
                }
                positions.add(distanceAdjusted)
                smoothedDistance = positions.average()

                lastDetection = smoothedDistance

                // Display distance
                Imgproc.putText(
                    undistorted,
                    "Distance: %.2f inches".format(smoothedDistance),
                    Point(50.0, 50.0),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    Scalar(255.0, 255.0, 255.0),
                    2
                )
            } else if (lastDetection != null) {
                // Use last known detection
                Imgproc.putText(
                    undistorted,
                    "Distance: %.2f inches (last known)".format(lastDetection),
                    Point(50.0, 50.0),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    Scalar(255.0, 255.0, 255.0),
                    2
                )
            }
        } else if (lastDetection != null) {
            // No contours found, use last known detection
            Imgproc.putText(
                undistorted,
                "Distance: %.2f inches (last known)".format(lastDetection),
                Point(50.0, 50.0),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1.0,
                Scalar(255.0, 255.0, 255.0),
                2
            )
        }

        // Send the undistorted image to the dashboard
        val bitmap = Bitmap.createBitmap(undistorted.cols(), undistorted.rows(), Bitmap.Config.ARGB_8888)
        Utils.matToBitmap(undistorted, bitmap)
        dashboard.sendImage(bitmap)

        // Return the processed frame
        return undistorted
    }
}