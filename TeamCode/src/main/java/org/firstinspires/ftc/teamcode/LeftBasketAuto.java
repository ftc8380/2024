package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Config
@Autonomous(group = "drive")
public class LeftBasketAuto extends OpMode {

    private MecanumDrive drive;

    private DcMotor armRotationMotor;
    private DcMotor armExtensionMotor;
    private Servo armClaw;

    private Trajectory startToBasket;
    private Trajectory forward;
    private Trajectory backward;

    private Trajectory grabOne;

    private Trajectory grabTwo;

    private Trajectory secondBasket;

    private Trajectory thirdBasket;

    private Trajectory finalBackwards;

    // For timing tiny waits instead of Thread.sleep()
    private double stepStartTime;
    private final double WAIT_TIME = 0.2; // 200 ms pause

    // Step counter to track progress without switch or enums
    private int step = 0;

    @Override
    public void init() {
        // Initialize hardware
        drive = new MecanumDrive(hardwareMap);

        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotation");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");
        armClaw = hardwareMap.get(Servo.class, "claw_grab");

        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Build trajectories
        startToBasket = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(9, 20), Math.toRadians(135))
                .build();

        forward = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
        backward = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .build();

        grabOne = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, -15), Math.toRadians(-135))
                .build();
        grabTwo = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30,-9), Math.toRadians((135)))
                .build();
        secondBasket = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30,15), Math.toRadians((135)))
                .build();

        thirdBasket = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30,9), Math.toRadians((135)))
                .build();
        finalBackwards = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(5,0), Math.toRadians(-135))
                .build();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Runs repeatedly after init() but before start
    }

    @Override
    public void start() {
        // Start the first trajectory
          drive.followTrajectoryAsync(startToBasket);
        step = 1; // Move to next step
        stepStartTime = getRuntime();

    }

    @Override
    public void loop() {
        // Update Road Runner
        drive.update();

        // Step 1: Wait for trajectoryOne to finish
        if (step == 1 && !drive.isBusy()) {
            // Move arm up

            setArmLength(5);
            stepStartTime = getRuntime();
            step = 2;
        }

        // Step 2: Wait ~0.2 seconds, then start trajectoryTwo
        if (step == 2 && armExtensionMotor.getCurrentPosition() >= 1900) {
            drive.followTrajectoryAsync(forward);
            stepStartTime = getRuntime();
            step = 3;
        }


        // Step 3: Wait ~0.2 seconds
        if (step == 3 && !drive.isBusy()) {
            // Close the claw
            armClaw.setPosition(0.2);
            stepStartTime = getRuntime();
            step = 4;
        }

        // Step 4: Wait ~0.2 seconds, then drive backwards
        if (step == 4 && getRuntime() - stepStartTime > WAIT_TIME) {
            drive.followTrajectoryAsync(backward);
            stepStartTime = getRuntime();
            step = 5;
        }

        // Step 5: Wait ~0.2 seconds for arm to move down
        if (step == 5 && !drive.isBusy() && getRuntime() - stepStartTime > WAIT_TIME ) {
            // Pull arm down
            setArmLength(0);
            stepStartTime = getRuntime();
            step = 6;
        }

        // Step 6: Wait for trajectoryThree to finish
        if (step == 6 && armExtensionMotor.getCurrentPosition() <= 100) {

            drive.followTrajectoryAsync(grabOne);


            stepStartTime = getRuntime();
            step = 7;
        }
        if (step == 7 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmAngle(0.2);

            stepStartTime = getRuntime();
            step = 8;
        }
        if (step == 8 && getRuntime() - stepStartTime > WAIT_TIME) {
            // grab
            armClaw.setPosition(0);
            stepStartTime = getRuntime();
            step = 9;
        }
        if (step == 9 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            setArmAngle(0);
            stepStartTime = getRuntime();
            step = 10;
        }
        if (step == 10 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            drive.followTrajectoryAsync(secondBasket);
            stepStartTime = getRuntime();
            step = 11;
        }
        if (step == 11 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmLength(5);
            stepStartTime = getRuntime();
            step = 12;
        }
        if (step == 12 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            drive.followTrajectoryAsync(forward);
            stepStartTime = getRuntime();
            step = 13;
        }
        if (step == 13 && !drive.isBusy()) {
            // Retract arm and open claw
            armClaw.setPosition(0.2);
            drive.followTrajectoryAsync(backward);
            stepStartTime = getRuntime();
            step = 14;
        }
        if (step == 14 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmLength(0);
            stepStartTime = getRuntime();
            step = 15;
        }
        if (step == 15 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            drive.followTrajectoryAsync(grabTwo);
            stepStartTime = getRuntime();
            step = 16;
        }
        if (step == 16 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmAngle(0.2);
            armClaw.setPosition(0.2);
            stepStartTime = getRuntime();
            step = 17;
        }
        if (step == 17 && armRotationMotor.getCurrentPosition() >= 0.19*5281.1) {
            // grab
            armClaw.setPosition(0);
            stepStartTime = getRuntime();
            step = 18;
        }
        if (step == 18 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            setArmAngle(0);
            stepStartTime = getRuntime();
            step = 19;
        }
        if (step == 19 && armRotationMotor.getCurrentPosition() >= 0.01*5281.1) {
            // Retract arm and open claw
            drive.followTrajectoryAsync(thirdBasket);
            stepStartTime = getRuntime();
            step = 20;
        }
        if (step == 20 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmLength(5);
            stepStartTime = getRuntime();
            step = 21;
        }
        if (step == 21 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            drive.followTrajectoryAsync(forward);
            stepStartTime = getRuntime();
            step = 22;
        }
        if (step == 22 && !drive.isBusy()) {
            // Retract arm and open claw
            armClaw.setPosition(0.2);
            drive.followTrajectoryAsync(backward);
            stepStartTime = getRuntime();
            step = 23;
        }
        if (step == 23 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmLength(0);
            stepStartTime = getRuntime();
            step = 24;
        }
        if (step == 24 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Retract arm and open claw
            drive.followTrajectoryAsync(finalBackwards);
            step = 25;
        }


        telemetry.addData("Step", step);
        telemetry.addData("Arm Ext. Ticks", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Arm Rot. Ticks", armRotationMotor.getCurrentPosition());
        telemetry.update();
    }

    /* Helper methods */
    private void setArmLength(double rotations) {
        int ticks = (int)(384.5 * rotations);
        // clamp ticks between 0 and 1100
        ticks = Math.max(0, Math.min(ticks, 2400));
        armExtensionMotor.setTargetPosition(ticks);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionMotor.setPower(1.0);
    }

    private void setArmAngle(double rotations) {
        int ticks = (int)(5281.1 * rotations);
        // clamp ticks between 0 and 1050
        ticks = Math.max(0, Math.min(ticks, 1050));
        armRotationMotor.setTargetPosition(ticks);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.8);
    }
}