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

    // Our single, simple state machine
    private StateMachine stateMachine = new StateMachine();

    @Override
    public void init() {
        // Hardware
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

        forward = drive.trajectoryBuilder(new Pose2d()).forward(5).build();
        backward = drive.trajectoryBuilder(new Pose2d()).back(5).build();

        grabOne = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, -15), Math.toRadians(-135))
                .build();

        grabTwo = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, -9), Math.toRadians(135))
                .build();

        secondBasket = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30, 15), Math.toRadians(135))
                .build();

        thirdBasket = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-30, 9), Math.toRadians(135))
                .build();

        finalBackwards = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(5, 0), Math.toRadians(-135))
                .build();

        //---------------------------------------------------------------------------------------
        // Now define each “step” as a State and chain them in order.
        //---------------------------------------------------------------------------------------

        // 1) Drive from start to basket
        stateMachine.addState(new TrajectoryState(drive, startToBasket));

        // 2) Extend arm to ~5 rotations until extension >= 1900
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

        // 3) Drive forward
        stateMachine.addState(new TrajectoryState(drive, forward));

        // 4) Close the claw (wait ~0.2s)
        stateMachine.addState(new TimedState(this, 0.2) {
            @Override
            public void onStart() {
                armClaw.setPosition(0.2);
            }
        });

        // 5) Drive backward
        stateMachine.addState(new TrajectoryState(drive, backward));

        // 6) Set arm length to 0 until extension <= 100
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

        // 7) Drive to grabOne
        stateMachine.addState(new TrajectoryState(drive, grabOne));

        // 8) Set arm angle to 0.2 (no wait needed, so we can do it quickly)
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmAngle(0.2);
            }
            @Override
            public void run() { }
            @Override
            public boolean isDone() {
                return true; // done immediately
            }
        });

        // 9) Wait 0.2s, then set claw to 0 (grab)
        stateMachine.addState(new TimedState(this, 0.2) {
            @Override
            public void onStart() {
                armClaw.setPosition(0);
            }
        });

        // 10) Return arm angle to 0
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmAngle(0);
            }
            @Override
            public void run() { }
            @Override
            public boolean isDone() {
                // If you want to wait until it actually finishes, check motor pos
                return true;
            }
        });

        // 11) Drive to secondBasket
        stateMachine.addState(new TrajectoryState(drive, secondBasket));

        // 12) Extend arm to 5
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(5);
            }
            @Override
            public void run() { }
            @Override
            public boolean isDone() {
                // If you want a threshold, check extension or return true immediately
                return true;
            }
        });

        // 13) Drive forward
        stateMachine.addState(new TrajectoryState(drive, forward));

        // 14) Close claw, then drive backward
        stateMachine.addState(new State() {
            private boolean doneClaw = false;
            @Override
            public void init() {
                // Close claw
                armClaw.setPosition(0.2);
                // Immediately start backward trajectory
                drive.followTrajectoryAsync(backward);
            }
            @Override
            public void run() {
                drive.update();
                if (!drive.isBusy()) doneClaw = true;
            }
            @Override
            public boolean isDone() {
                return doneClaw;
            }
        });

        // 15) Arm length to 0
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(0);
            }
            @Override
            public void run() {}
            @Override
            public boolean isDone() { return true; }
        });

        // 16) Drive to grabTwo
        stateMachine.addState(new TrajectoryState(drive, grabTwo));

        // 17) Lift arm angle to 0.2, claw near 0.2
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmAngle(0.2);
                armClaw.setPosition(0.2);
            }
            @Override
            public void run() {}
            @Override
            public boolean isDone() {
                // Wait until angle is at 0.2? For simplicity just do:
                return true;
            }
        });

        // 18) Claw to 0 (grab)
        stateMachine.addState(new TimedState(this, 0.2) {
            @Override
            public void onStart() {
                armClaw.setPosition(0);
            }
        });

        // 19) Arm angle to 0
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmAngle(0);
            }
            @Override
            public void run() {}
            @Override
            public boolean isDone() {
                return true;
            }
        });

        // 20) Drive to thirdBasket
        stateMachine.addState(new TrajectoryState(drive, thirdBasket));

        // 21) Arm length to 5
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(5);
            }
            @Override
            public void run() {}
            @Override
            public boolean isDone() {
                return true;
            }
        });

        // 22) Drive forward
        stateMachine.addState(new TrajectoryState(drive, forward));

        // 23) Claw close, then drive backward
        stateMachine.addState(new State() {
            private boolean doneBack = false;
            @Override
            public void init() {
                armClaw.setPosition(0.2);
                drive.followTrajectoryAsync(backward);
            }
            @Override
            public void run() {
                drive.update();
                if (!drive.isBusy()) doneBack = true;
            }
            @Override
            public boolean isDone() {
                return doneBack;
            }
        });

        // 24) Arm length to 0
        stateMachine.addState(new State() {
            @Override
            public void init() {
                setArmLength(0);
            }
            @Override
            public void run() {}
            @Override
            public boolean isDone() { return true; }
        });

        // 25) Drive finalBackwards (end)
        stateMachine.addState(new TrajectoryState(drive, finalBackwards));

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // If needed
    }

    @Override
    public void start() {
        // We do nothing special here. The StateMachine starts at state #0 automatically.
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
        int ticks = (int)(384.5 * rotations);
        // clamp ticks between 0 and 2400
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