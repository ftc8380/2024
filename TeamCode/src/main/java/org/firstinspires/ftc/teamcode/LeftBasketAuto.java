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
import org.firstinspires.ftc.teamcode.util.TrajectoryState;
import org.firstinspires.ftc.teamcode.util.TimedState;

@Config
@Autonomous(group = "drive")
public class LeftBasketAuto extends OpMode {

    // CONFIG VARIABLES FOR TRAJECTORIES
    // Starting Pose
    public static double startPoseX = 36;
    public static double startPoseY = 60;
    public static double startPoseHeadingDeg = -90;

    // Basket Target Pose (for the spline)
    public static double basketX = 48;
    public static double basketY = 48;
    public static double basketSplineHeadingDeg = 45;

    // Distances for forward and backward movements (in inches or your field unit)
    public static double forwardDistance = 4.5;
    public static double backwardDistance = 5.0;

    private MecanumDrive drive;

    private DcMotor armRotationMotor;
    private DcMotor armExtensionMotor;
    private Servo armClaw;

    private Trajectory startToBasket;
    private Trajectory forward;
    private Trajectory backward;
    // Our single, simple state machine
    private StateMachine stateMachine = new StateMachine();

    @Override
    public void init() {
        // Hardware initialization
        drive = new MecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeadingDeg)));
        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotation");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");
        armClaw = hardwareMap.get(Servo.class, "claw_grab");

        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set servo scale range to 0.0 to 0.05
        armClaw.scaleRange(0.0, 0.05);

        // Build trajectory from start to basket (spline)
        startToBasket = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(basketX, basketY), Math.toRadians(basketSplineHeadingDeg))
                .build();

        // Calculate the forward target pose (robot-relative forward)
        Pose2d currentPoseAfterSpline = startToBasket.end();
        double heading = currentPoseAfterSpline.getHeading();
        double forwardTargetX = currentPoseAfterSpline.getX() + forwardDistance * Math.cos(heading);
        double forwardTargetY = currentPoseAfterSpline.getY() + forwardDistance * Math.sin(heading);
        Pose2d forwardTargetPose = new Pose2d(forwardTargetX, forwardTargetY, heading);

        // Build forward trajectory using an absolute target pose
        forward = drive.trajectoryBuilder(currentPoseAfterSpline)
                .lineToLinearHeading(forwardTargetPose)
                .build();

        // Calculate the backward target pose (robot-relative backward)
        Pose2d currentPoseAfterForward = forward.end();
        double backwardTargetX = currentPoseAfterForward.getX() - backwardDistance * Math.cos(heading);
        double backwardTargetY = currentPoseAfterForward.getY() - backwardDistance * Math.sin(heading);
        Pose2d backwardTargetPose = new Pose2d(backwardTargetX, backwardTargetY, heading);

        // Build backward trajectory using an absolute target pose
        backward = drive.trajectoryBuilder(currentPoseAfterForward)
                .lineToLinearHeading(backwardTargetPose)
                .build();

        //---------------------------------------------------------------------------------------
        // Define each “step” as a State and chain them in order.
        //---------------------------------------------------------------------------------------

        // 1) Drive from start to basket (spline path)
        stateMachine.addState(new TrajectoryState(drive, startToBasket));

        // 2) Extend arm to ~5 rotations until extension >= 1900 ticks
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(5);
            }
            @Override
            public void run() { /* do nothing */ }
            @Override
            public boolean isDone() {
                return armExtensionMotor.getCurrentPosition() >= 1900;
            }
        });

        // 3) Drive forward (robot-relative, computed as an absolute pose)
        stateMachine.addState(new TrajectoryState(drive, forward));

        // 4) Close the claw (wait ~0.2s) by setting the servo to 0.0 (within the scaled range)
        stateMachine.addState(new TimedState(LeftBasketAuto.this, 0.2) {
            @Override
            public void onStart() {
                armClaw.setPosition(0.0);
            }
        });

        // 5) Drive backward (computed absolute pose)
        stateMachine.addState(new TrajectoryState(drive, backward));

        // 6) Retract the arm to 0 until extension <= 100 ticks
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(0);
            }
            @Override
            public void run() { /* do nothing */ }
            @Override
            public boolean isDone() {
                return armExtensionMotor.getCurrentPosition() <= 100;
            }
        });

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Optional: Code to run repeatedly during the init phase
    }

    @Override
    public void start() {
        // The StateMachine starts at state #0 automatically.
    }

    @Override
    public void loop() {
        // Let the state machine handle everything
        stateMachine.update();

        // Telemetry
        telemetry.addData("Machine Done?", stateMachine.isFinished());
        telemetry.addData("Arm Ext. Ticks", armExtensionMotor.getCurrentPosition());
        telemetry.addData("Arm Rot. Ticks", armRotationMotor.getCurrentPosition());
        telemetry.update();
    }

    //-------------------------------------------------------------------------
    // Helper methods for arm/claw
    //-------------------------------------------------------------------------
    private void setArmLength(double rotations) {
        int ticks = (int) (384.5 * rotations);
        // Clamp ticks between 0 and 2400
        ticks = Math.max(0, Math.min(ticks, 2400));
        armExtensionMotor.setTargetPosition(ticks);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtensionMotor.setPower(1.0);
    }

    private void setArmAngle(double rotations) {
        int ticks = (int) (5281.1 * rotations);
        // Clamp ticks between 0 and 1050
        ticks = Math.max(0, Math.min(ticks, 1050));
        armRotationMotor.setTargetPosition(ticks);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setPower(0.8);
    }
}