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
public class RightMultiSample extends OpMode {

    //================== Configurable Parameters ==================
    // Starting pose parameters
    public static double START_X = 24;
    // Y position is calculated as -72 + (17.375 / 2)
    public static double START_Y = -72 + (17.375 / 2);
    public static double START_HEADING_DEGREES = 90;

    // Trajectory distances and targets
    public static double TRAJ_FORWARD_TARGET_X = 0;
    public static double TRAJ_FORWARD_TARGET_Y = -32;
    public static double TRAJ_FORWARD_TARGET_HEADING_DEGREES = 90;

    public static double TRAJ_TWO_DISTANCE = 6;
    public static double TRAJ_THREE_BACK_DISTANCE = 30;
    public static double TRAJ_FOUR_STRAFE_RIGHT_DISTANCE = 48;

    // Arm configurations (in rotations)
    public static double ARM_LENGTH_INITIAL = 3.6;
    public static double ARM_LENGTH_ADJUSTED = 1.5;

    // Motor conversion factors and limits
    public static double ARM_LENGTH_SCALE_FACTOR = 384.5;  // ticks per rotation for arm extension
    public static int ARM_LENGTH_MAX_TICKS = 1100;

    public static double ARM_ANGLE_SCALE_FACTOR = 5281.1;  // ticks per rotation for arm rotation
    public static int ARM_ANGLE_MAX_TICKS = 1050;

    //================== End Configurable Parameters ==================

    private MecanumDrive drive;
    private DcMotor armRotationMotor;
    private DcMotor armExtensionMotor;
    private Servo armClaw;

    private Trajectory trajectoryForward;
    private Trajectory trajectoryTwo;
    private Trajectory trajectoryThree;
    private Trajectory trajectoryFour;

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

        armClaw.scaleRange(0.0, 0.05);

        // Set the starting pose using configurable parameters.
        drive.setPoseEstimate(new Pose2d(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEGREES)
        ));

        // Build trajectories using configurable distances.
        trajectoryForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(
                        new Vector2d(TRAJ_FORWARD_TARGET_X, TRAJ_FORWARD_TARGET_Y),
                        Math.toRadians(TRAJ_FORWARD_TARGET_HEADING_DEGREES)
                )
                .build();

        trajectoryTwo = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(TRAJ_TWO_DISTANCE)
                .build();

        trajectoryThree = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(TRAJ_THREE_BACK_DISTANCE)
                .build();

        trajectoryFour = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(TRAJ_FOUR_STRAFE_RIGHT_DISTANCE)
                .build();

        // --- Add states to the StateMachine ---

        // 1) Drive forward trajectoryForward.
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

        // 2) Raise arm a bit (and wait ~1.5s)
        stateMachine.addState(new TimedState(RightMultiSample.this, 1.5) {
            @Override
            public void onStart() {
                setArmLength(ARM_LENGTH_INITIAL);
                armClaw.setPosition(0.9);
            }
        });

        // 3) Drive forward trajectoryTwo.
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

        // 4) Adjust arm length (and wait ~1.2s)
        stateMachine.addState(new TimedState(this, 1.2) {
            @Override
            public void onStart() {
                setArmLength(ARM_LENGTH_ADJUSTED);
            }
        });

        // 5) Close claw (and wait ~0.2s)
        stateMachine.addState(new TimedState(this, 0.2) {
            @Override
            public void onStart() {
                armClaw.setPosition(0);
            }
        });

        // 6) Drive forward trajectoryThree.
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

        // 7) Drive forward trajectoryFour.
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryFour);
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
        // Update the StateMachine each loop.
        stateMachine.update();

        // Show some status.
        telemetry.addData("IsFinished?", stateMachine.isFinished());
        telemetry.addData("Arm Ext. Ticks", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Arm Rot. Ticks", armRotationMotor.getCurrentPosition());
        telemetry.update();
    }

    /* Helper methods */
    private void setArmLength(double rotations) {
        int ticks = (int)(ARM_LENGTH_SCALE_FACTOR * rotations);
        // Clamp ticks between 0 and ARM_LENGTH_MAX_TICKS.
        ticks = Math.max(0, Math.min(ticks, ARM_LENGTH_MAX_TICKS));
        armExtensionMotor.setTargetPosition(ticks);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionMotor.setPower(1.0);
    }

    private void setArmAngle(double rotations) {
        int ticks = (int)(ARM_ANGLE_SCALE_FACTOR * rotations);
        // Clamp ticks between 0 and ARM_ANGLE_MAX_TICKS.
        ticks = Math.max(0, Math.min(ticks, ARM_ANGLE_MAX_TICKS));
        armRotationMotor.setTargetPosition(ticks);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.8);
    }
}
