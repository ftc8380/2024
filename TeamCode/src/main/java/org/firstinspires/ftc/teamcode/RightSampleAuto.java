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
import org.firstinspires.ftc.teamcode.util.State;
import org.firstinspires.ftc.teamcode.util.StateMachine;
import org.firstinspires.ftc.teamcode.util.TimedState;

@Config
@Autonomous(group = "drive")
public class RightSampleAuto extends OpMode {

    private MecanumDrive drive;
    private DcMotor armRotationMotor;
    private DcMotor armExtensionMotor;
    private Servo armClaw;

    private Trajectory trajectoryForward;
    private Trajectory trajectoryTwo;
    private Trajectory trajectoryThree;

    // Our minimal state machine
    private StateMachine stateMachine = new StateMachine();

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);

        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotation");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");
        armClaw = hardwareMap.get(Servo.class, "claw_grab");

        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Build trajectories
        trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(18, 55), Math.toRadians(0))
                .build();

        trajectoryTwo = drive.trajectoryBuilder(new Pose2d())
                .forward(11)
                .build();

        trajectoryThree = drive.trajectoryBuilder(new Pose2d())
                .back(10)
                .strafeRight(85)
                .back(21)
                .build();

        // --- Add states to the StateMachine ---

        // 1) Drive forward
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryForward);
            }

            @Override
            public void run() {
                drive.update();
            }

            @Override
            public boolean isDone() {
                return !drive.isBusy();
            }
        });

        // 2) Raise arm a bit (and wait ~0.2s)
        stateMachine.addState(new TimedState(this,0.2) {
            @Override
            public void onStart() {
                setArmLength(4);
            }
        });

        // 3) Drive forward second trajectory
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryTwo);
            }

            @Override
            public void run() {
                drive.update();
            }

            @Override
            public boolean isDone() {
                return !drive.isBusy();
            }
        });

        // 4) Adjust arm length (and wait ~2s)
        stateMachine.addState(new TimedState(this,2.0) {
            @Override
            public void onStart() {
                setArmLength(1.5);
            }
        });

        // 5) Close claw (and wait ~0.2s)
        stateMachine.addState(new TimedState(this,0.2) {
            @Override
            public void onStart() {
                armClaw.setPosition(0.2);
            }
        });

        // 6) Move arm down (and wait ~0.2s)
        stateMachine.addState(new TimedState(this,0.2) {
            @Override
            public void onStart() {
                setArmAngle(0);
            }
        });

        // 7) Drive backward third trajectory
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryThree);
            }

            @Override
            public void run() {
                drive.update();
            }

            @Override
            public boolean isDone() {
                return !drive.isBusy();
            }
        });

        // 8) Retract arm & open claw (done)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(-100);
                setArmAngle(0);
                armClaw.setPosition(0);
            }
            @Override
            public void run() { /* do nothing */ }
            @Override
            public boolean isDone() { return true; }
        });

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Runs repeatedly after init() but before start
    }

    @Override
    public void start() {
        // No manual step = 1 here—StateMachine does that for us
    }

    @Override
    public void loop() {
        // Update the StateMachine each loop
        stateMachine.update();

        // Show some status
        telemetry.addData("IsFinished?", stateMachine.isFinished());
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