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

    private Trajectory trajectoryOne;
    private Trajectory trajectoryTwo;
    private Trajectory trajectoryThree;

    private Trajectory trajectoryFour;

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
        trajectoryOne = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(9, 30), Math.toRadians(135))
                .build();

        trajectoryTwo = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
        trajectoryThree = drive.trajectoryBuilder(new Pose2d())
                .back(5)
                .build();

        trajectoryFour = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-6,10), Math.toRadians(-135))
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
        drive.followTrajectoryAsync(trajectoryOne);
        step = 1; // Move to next step
        stepStartTime = getRuntime();
    }

    @Override
    public void loop() {
        // Update Road Runner
        drive.update();

        // Step 1: Wait for trajectoryOne to finish
        if (step == 1 && !drive.isBusy()) {
            // Move arm slightly up and out
            setArmAngle(0.05);
            setArmLength(5);
            stepStartTime = getRuntime();
            step = 2;
        }

        // Step 2: Wait ~0.2 seconds, then start trajectoryTwo
        if (step == 2 && getRuntime() - stepStartTime > WAIT_TIME) {
            drive.followTrajectoryAsync(trajectoryTwo);
            step = 3;
        }


        // Step 3: Wait ~0.2 seconds
        if (step == 3 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Close the claw
            armClaw.setPosition(0.2);
            stepStartTime = getRuntime();
            step = 4;
        }

        // Step 4: Wait ~0.2 seconds, then pull arm down before trajectory 3
        if (step == 4 && getRuntime() - stepStartTime > WAIT_TIME) {
            setArmAngle(0);
            stepStartTime = getRuntime();
            step = 5;
        }

        // Step 5: Wait ~0.2 seconds for arm to move down
        if (step == 5 && getRuntime() - stepStartTime > WAIT_TIME) {
            // Start trajectoryThree
            drive.followTrajectoryAsync(trajectoryThree);
            step = 6;
        }

        // Step 6: Wait for trajectoryThree to finish
        if (step == 6 && !drive.isBusy()) {
            // Retract arm and open claw
            setArmLength(-100); // will clamp to 0
            setArmAngle(0);
            armClaw.setPosition(0);
            step = 7;
        }
        if (step == 7 && getRuntime() - stepStartTime > WAIT_TIME) {
        // Retract arm and open claw
        drive.followTrajectoryAsync(trajectoryFour);
        step = 8;
    }

        // Step 8: Done
        // No further actions needed.

        telemetry.addData("Step", step);
        telemetry.addData("Arm Ext. Ticks", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Arm Rot. Ticks", armRotationMotor.getCurrentPosition());
        telemetry.update();
    }

    /* Helper methods */
    private void setArmLength(double rotations) {
        int ticks = (int)(384.5 * rotations);
        // clamp ticks between 0 and 1100
        ticks = Math.max(0, Math.min(ticks, 1100));
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