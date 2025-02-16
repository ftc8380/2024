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
public class ScoreOneSample extends OpMode {

    //================== Configurable Parameters ==================
    // Starting pose (matches MeepMeep simulation)
    public static double START_X = 24;
    // Calculated as -72 + (17.375/2) = -63.3125
    public static double START_Y = -72 + (17.375 / 2);
    public static double START_HEADING_DEGREES = 90;

    // Trajectory 1: spline to target (0, -32) with heading 90°
    public static double TRAJ1_TARGET_X = 0;
    public static double TRAJ1_TARGET_Y = -32;
    public static double TRAJ1_TARGET_HEADING_DEGREES = 90;

    // Trajectory 2: forward distance (changed to 4 to match simulation)
    public static double TRAJ2_DISTANCE = 4;

    // Trajectory 3: back distance (15 units)
    public static double TRAJ3_BACK_DISTANCE = 15;

    // Trajectory 3.5: instead of splineTo, we use strafeTo (to (36, -36))
    public static double TRAJ3_5_TARGET_X = 36;
    public static double TRAJ3_5_TARGET_Y = -36;

    // Trajectory 4: now uses strafeTo to (36, -8.7)
    public static double TRAJ4_TARGET_X = 36;
    public static double TRAJ4_TARGET_Y = -8.7;

    // Trajectory 5: strafe to (42, -8.7)
    public static double TRAJ5_TARGET_X = 42;
    public static double TRAJ5_TARGET_Y = -8.7;

    // Trajectory 6: line to (42, -60)
    public static double TRAJ6_TARGET_X = 42;
    public static double TRAJ6_TARGET_Y = -60;

    // Trajectory 7: line to (42, -40)
    public static double TRAJ7_TARGET_X = 42;
    public static double TRAJ7_TARGET_Y = -40;

    // Trajectory 8: line to (42, -56)
    public static double TRAJ8_TARGET_X = 42;
    public static double TRAJ8_TARGET_Y = -56;

    // Trajectory 9: line to (42, -63.2)  <-- Note the negative value
    public static double TRAJ9_TARGET_X = 42;
    public static double TRAJ9_TARGET_Y = -63.2;

    // Trajectory 10: back 5 units then strafe to (3, -40)
    public static double TRAJ10_BACK_DISTANCE = 5;
    public static double TRAJ10_TARGET_X = 3;
    public static double TRAJ10_TARGET_Y = -40;
    // (Heading is no longer used since we change to strafeTo)

    // Trajectory 11: strafe to (36, -60) for parking
    public static double TRAJ11_TARGET_X = 36;
    public static double TRAJ11_TARGET_Y = -60;

    // Arm parameters (rotations)
    public static double ARM_LENGTH_INITIAL = 3.6;
    public static double ARM_LENGTH_ADJUSTED = 1.5;

    public static double ARM_ANGLE_INITIAL = 0.1; // For lowering arm in state 12
    public static double ARM_ANGLE_FINAL = 0;     // For returning arm in state 14

    // Time delays (in seconds) for timed states
    public static double TIME_RAISE_ARM = 1.5;
    public static double TIME_ARM_ADJUST = 1.2;
    public static double TIME_CLOSE_CLAW_SHORT = 0.2;
    public static double TIME_WAIT_AFTER_TURN = 3;
    public static double TIME_LOWER_ARM = 0.7;
    public static double TIME_CLOSE_CLAW_LONG = 0.7;
    public static double TIME_ARM_RETURN = 0.8;

    // Arm conversion factors and limits
    public static double ARM_LENGTH_SCALE_FACTOR = 384.5;  // ticks per rotation for arm extension
    public static int ARM_LENGTH_MAX_TICKS = 1100;

    public static double ARM_ANGLE_SCALE_FACTOR = 5281.1;    // ticks per rotation for arm rotation
    public static int ARM_ANGLE_MAX_TICKS = 1050;
    //================== End Configurable Parameters ==================

    private MecanumDrive drive;
    private DcMotor armRotationMotor;
    private DcMotor armExtensionMotor;
    private Servo armClaw;

    private Trajectory trajectoryForward;
    private Trajectory trajectoryTwo;
    private Trajectory trajectoryThree;
    private Trajectory trajectoryThreeAndAHalf;
    private Trajectory trajectoryFour;
    private Trajectory trajectoryFive;
    private Trajectory trajectorySix;
    private Trajectory trajectorySeven;
    private Trajectory trajectoryEight;
    private Trajectory trajectoryNine;
    private Trajectory trajectoryTen;
    private Trajectory trajectoryEleven;

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

        // Set starting pose using configurable variables.
        drive.setPoseEstimate(new Pose2d(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEGREES)
        ));

        // Build trajectories using configurable parameters.
        trajectoryForward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(
                        new Vector2d(TRAJ1_TARGET_X, TRAJ1_TARGET_Y),
                        Math.toRadians(TRAJ1_TARGET_HEADING_DEGREES)
                )
                .build();

        trajectoryTwo = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(TRAJ2_DISTANCE)
                .build();

        trajectoryThree = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(TRAJ3_BACK_DISTANCE)
                .build();

        // Instead of splineTo, we now use strafeTo for a straight lateral move.
        trajectoryThreeAndAHalf = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(TRAJ3_5_TARGET_X, TRAJ3_5_TARGET_Y))
                .build();

        // Similarly, use strafeTo for trajectoryFour.
        trajectoryFour = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(TRAJ4_TARGET_X, TRAJ4_TARGET_Y))
                .build();

        trajectoryFive = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(TRAJ5_TARGET_X, TRAJ5_TARGET_Y))
                .build();

        trajectorySix = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(TRAJ6_TARGET_X, TRAJ6_TARGET_Y))
                .build();

        trajectorySeven = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(TRAJ7_TARGET_X, TRAJ7_TARGET_Y))
                .build();

        trajectoryEight = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(TRAJ8_TARGET_X, TRAJ8_TARGET_Y))
                .build();

        trajectoryNine = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(TRAJ9_TARGET_X, TRAJ9_TARGET_Y))
                .build();

        // TrajectoryTen: back 5 units then strafe to (3, -40)
        trajectoryTen = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(TRAJ10_BACK_DISTANCE)
                .strafeTo(new Vector2d(TRAJ10_TARGET_X, TRAJ10_TARGET_Y))
                .build();

        trajectoryEleven = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(TRAJ11_TARGET_X, TRAJ11_TARGET_Y))
                .build();

        // --- Add states to the StateMachine ---
        // 1) Drive forward (trajectoryForward)
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

        // 2) Raise arm a bit and open claw (wait TIME_RAISE_ARM)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_RAISE_ARM) {
            @Override
            public void onStart() {
                setArmLength(ARM_LENGTH_INITIAL);
                armClaw.setPosition(0.9);
            }
        });

        // 3) Drive forward (trajectoryTwo) -- now 4 units
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

        // 4) Adjust arm length (wait TIME_ARM_ADJUST)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_ARM_ADJUST) {
            @Override
            public void onStart() {
                setArmLength(ARM_LENGTH_ADJUSTED);
            }
        });

        // 5) Close claw (wait TIME_CLOSE_CLAW_SHORT)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_CLOSE_CLAW_SHORT) {
            @Override
            public void onStart() {
                armClaw.setPosition(0);
            }
        });

        // 6) Drive backward (trajectoryThree)
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

        // 7) Strafe to new point (trajectoryThreeAndAHalf)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryThreeAndAHalf);
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

        // 8) Strafe to point (trajectoryFour)
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

        // 9) Strafe to point (trajectoryFive)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryFive);
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

        // 9.5) New state: Turn +90° after trajectoryFive
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.turn(Math.toRadians(90));
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

        // 10) Line to point (trajectorySix)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectorySix);
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

        // 11) Line to point (trajectorySeven)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectorySeven);
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

        // 12) Turn +90° (updated from -90°)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.turn(Math.toRadians(90));
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

        // 13) Wait (TIME_WAIT_AFTER_TURN)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_WAIT_AFTER_TURN) {
            @Override
            public void onStart() {
                // Optionally, you can add an extra claw command here if needed.
                armClaw.setPosition(0);
            }
        });

        // 14) Drive line to point (trajectoryEight)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryEight);
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

        // 15) Lower arm (wait TIME_LOWER_ARM)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_LOWER_ARM) {
            @Override
            public void onStart() {
                setArmAngle(ARM_ANGLE_INITIAL);
            }
        });

        // 16) Close claw (wait TIME_CLOSE_CLAW_LONG)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_CLOSE_CLAW_LONG) {
            @Override
            public void onStart() {
                armClaw.setPosition(0.9);
            }
        });

        // 17) Drive line and rotate arm back (wait TIME_ARM_RETURN) (trajectoryNine)
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_ARM_RETURN) {
            @Override
            public void onStart() {
                setArmAngle(ARM_ANGLE_FINAL);
            }
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryNine);
            }
            @Override
            public void run() {
                drive.update();
            }
        });

        // 18) Drive back (trajectoryTen) -- now back 5 then strafe to (3, -40)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryTen);
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

        // (Optional) The remaining states for a second cycle or parking can be adjusted similarly.
        // For example, here we simply park using trajectoryEleven.
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_RAISE_ARM) {
            @Override
            public void onStart() {
                setArmLength(ARM_LENGTH_INITIAL);
                armClaw.setPosition(0.9);
            }
        });
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
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_ARM_ADJUST) {
            @Override
            public void onStart() {
                setArmLength(ARM_LENGTH_ADJUSTED);
            }
        });
        stateMachine.addState(new TimedState(ScoreOneSample.this, TIME_CLOSE_CLAW_SHORT) {
            @Override
            public void onStart() {
                armClaw.setPosition(0);
            }
        });
        stateMachine.addState(new State() {
            @Override
            public void init() {
                drive.followTrajectoryAsync(trajectoryEleven);
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
        // No manual step—StateMachine handles sequencing
    }

    @Override
    public void loop() {
        stateMachine.update();

        telemetry.addData("IsFinished?", stateMachine.isFinished());
        telemetry.addData("Arm Ext. Ticks", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Arm Rot. Ticks", armRotationMotor.getCurrentPosition());
        telemetry.update();
    }

    /* Helper methods */
    private void setArmLength(double rotations) {
        int ticks = (int)(ARM_LENGTH_SCALE_FACTOR * rotations);
        // Clamp ticks between 0 and ARM_LENGTH_MAX_TICKS
        ticks = Math.max(0, Math.min(ticks, ARM_LENGTH_MAX_TICKS));
        armExtensionMotor.setTargetPosition(ticks);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionMotor.setPower(1.0);
    }

    private void setArmAngle(double rotations) {
        int ticks = (int)(ARM_ANGLE_SCALE_FACTOR * rotations);
        // Clamp ticks between 0 and ARM_ANGLE_MAX_TICKS
        ticks = Math.max(0, Math.min(ticks, ARM_ANGLE_MAX_TICKS));
        armRotationMotor.setTargetPosition(ticks);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.8);
    }
}
