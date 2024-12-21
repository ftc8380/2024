package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using MecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class AutoTest extends LinearOpMode {


    private DcMotor armRotationMotor;
    private DcMotor armExtensionMotor;
    private Servo armClaw;

    private void setArmLength(double rotations){
        int ticks = (int)(384.5 * rotations);
        ticks = Math.max(0, Math.min(ticks, 1100));
        armExtensionMotor.setTargetPosition(ticks);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionMotor.setPower(1.0);
    }
    private void setArmAngle(double rotations){
        int ticks = (int)(5281.1 * rotations);
        ticks = Math.max(0, Math.min(ticks, 1050));
        armRotationMotor.setTargetPosition(ticks);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.8);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotation");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");
        armClaw = hardwareMap.get(Servo.class, "claw_grab");
        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(18,50), Math.toRadians(0))
                .build();

        Trajectory trajectoryTwo = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
        Trajectory trajectoryThree = drive.trajectoryBuilder(new Pose2d())
                .back(12)
                .build();


        waitForStart();
        this.telemetry.addData("Arm length", armExtensionMotor.getCurrentPosition());
        this.telemetry.update();
        drive.followTrajectory(trajectoryForward);
        setArmAngle(0.05);
        setArmLength(3.5);
        Thread.sleep(200);
        drive.followTrajectory(trajectoryTwo);
        Thread.sleep(200);
        setArmLength(2.9);
        Thread.sleep(200);
        armClaw.setPosition(0.2);
        Thread.sleep(200);
        drive.followTrajectory(trajectoryThree);
        setArmLength(-100);
        setArmAngle(0);
        armClaw.setPosition(0);



    }
}