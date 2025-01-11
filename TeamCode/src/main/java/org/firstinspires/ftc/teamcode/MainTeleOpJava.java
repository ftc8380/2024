package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

@TeleOp
public class MainTeleOpJava extends OpMode {
    // Declare hardware variables
    private DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight, armRotationMotor, armExtensionMotor;
    private DcMotorSimple hangingArm;
    private BNO055IMU imu;
    private Servo armClaw;

    // Variables for scaling and gamepad state
    private double[] speedModes = {0.3, 0.55, 0.8, 1.0};
    private int currentSpeedModeIndex = 1; // default to 0.55
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private double dpadY, dpadX;

    int maxExtension = 1100;

    @Override
    public void init() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "LF");
        motorBackLeft = hardwareMap.get(DcMotor.class, "LR");
        motorFrontRight = hardwareMap.get(DcMotor.class, "RF");
        motorBackRight = hardwareMap.get(DcMotor.class, "RR");
        armClaw = hardwareMap.get(Servo.class, "claw_grab");
        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotation");
        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");
        hangingArm = hardwareMap.get(DcMotor.class, "hanging");

        armClaw.scaleRange(-0.1, 0.1);

        // Set all motors to run using encoders
        List<DcMotor> motors = Arrays.asList(
                motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight
        );
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setTargetPosition(0);
        armExtensionMotor.setPower(1);
        armRotationMotor.setTargetPosition(0);
        armRotationMotor.setPower(1);
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reverse direction of left-side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        //ARM ROTATION
        if (gamepad2.back) {
            armClaw.setPosition(0.3);
            armRotationMotor.setTargetPosition(1100);
//            armRotationMotor.setPower(0.5);
        }
        //arm goes up
        else if (gamepad2.start) {
            armRotationMotor.setTargetPosition(20);
//            armRotationMotor.setPower(-0.6);
        }
        else if (gamepad2.right_bumper) {
            armExtensionMotor.setTargetPosition(945);
            armRotationMotor.setTargetPosition(0);
        }
        else if (gamepad2.left_bumper) {
            while(armExtensionMotor.getCurrentPosition() >= 450) {
                armExtensionMotor.setTargetPosition(440);
            }
            while(armClaw.getPosition() <= 0.28) {
                armClaw.setPosition(0.3);
            }

        }

//        if (gamepad2.left_stick_y != 0) {
//            armRotationMotor.setPower(gamepad2.left_stick_y / 2);
//        } else armRotationMotor.setPower(0.0);
        if(gamepad2.left_stick_y != 0) {
            armRotationMotor.setTargetPosition(Math.max(Math.min((armRotationMotor.getCurrentPosition() - (int) (gamepad2.left_stick_y * 70)), 1300), 0));
        }

        //ARM EXTENSION
        if (armRotationMotor.getCurrentPosition() < 250){
            maxExtension = 2400;
        }
        else maxExtension = 1100;
        if (gamepad2.right_stick_y != 0) {
            armExtensionMotor.setTargetPosition(Math.max(Math.min((armExtensionMotor.getCurrentPosition() - (int) (gamepad2.right_stick_y * 70)), maxExtension), 0));
        }

        // CLAW
        if (gamepad2.x) {
            armClaw.setPosition(0);
        }
        if (gamepad2.b) {
            armClaw.setPosition(0.3);
        }


        // HANGING
        hangingArm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


        // Overly complicated speed mode adjustment logic
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;


        //change this this is wack
        int adjustment = (currentDpadUp ? 1 : 0) - (currentDpadDown ? 1 : 0);
        boolean stateChanged = currentDpadUp != prevDpadUp || currentDpadDown != prevDpadDown;

        if (stateChanged) {
            int tentativeIndex = currentSpeedModeIndex;
            for (int i = 0; i < Math.abs(adjustment); i++) {
                if (adjustment > 0 && tentativeIndex < speedModes.length - 1) {
                    tentativeIndex++;
                } else if (adjustment < 0 && tentativeIndex > 0) {
                    tentativeIndex--;
                }
            }
            currentSpeedModeIndex = tentativeIndex;
        }

        // Update previous button states
        prevDpadUp = currentDpadUp;
        prevDpadDown = currentDpadDown;

        double speedScale = speedModes[currentSpeedModeIndex];

        double forwardTrigger = gamepad1.right_trigger;
        double backwardTrigger = gamepad1.left_trigger;

        double botOrientedY = 0.0;
        if (forwardTrigger > 0) {
            botOrientedY = forwardTrigger;
        } else if (backwardTrigger > 0) {
            botOrientedY = -backwardTrigger;
        }

        if (gamepad2.dpad_up){
            dpadY = -0.2;
        } else if (gamepad2.dpad_down) {
            dpadY = 0.2;
        }
        else dpadY = 0;
        if (gamepad2.dpad_right){
            dpadX = 0.2;
        } else if (gamepad2.dpad_left) {
            dpadX = -0.2;
        }
        else dpadX = 0;

        // NORMAL FIELD-ORIENTED JOYSTICK MOVEMENT
        double y = -(gamepad1.left_stick_y + dpadY);
        double x = (gamepad1.left_stick_x + dpadX) * 1.1;
        double rx = gamepad1.right_stick_x;

        double botHeadingRadians = -imu.getAngularOrientation().firstAngle;

        // Apply field-centric transformation for joystick movement
        double rotX = x * Math.cos(botHeadingRadians) - y * Math.sin(botHeadingRadians);
        double rotY = x * Math.sin(botHeadingRadians) + y * Math.cos(botHeadingRadians);

        // Combine bot-oriented trigger control with field-oriented joystick control
        double finalY = botOrientedY != 0.0 ? botOrientedY : rotY;

        double denominator = Math.max(Math.abs(finalY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double frontLeftPower = (finalY + rotX + rx) / denominator;
        double backLeftPower = (finalY - rotX + rx) / denominator;
        double frontRightPower = (finalY - rotX - rx) / denominator;
        double backRightPower = (finalY + rotX - rx) / denominator;

        boolean isTurboMode = gamepad1.right_bumper;
        double adjustedSpeedScale = isTurboMode ? 1.0 : speedScale;

        frontLeftPower *= adjustedSpeedScale;
        backLeftPower *= adjustedSpeedScale;
        frontRightPower *= adjustedSpeedScale;
        backRightPower *= adjustedSpeedScale;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        // Log motor positions to telemetry
    /*    telemetry.addData("Front Left Position", motorFrontLeft.getCurrentPosition());
        telemetry.addData("Back Left Position", motorBackLeft.getCurrentPosition());
        telemetry.addData("Front Right Position", motorFrontRight.getCurrentPosition());
        telemetry.addData("Back Right Position", motorBackRight.getCurrentPosition()); */
        telemetry.addData("Arm Rotation Position", armRotationMotor.getCurrentPosition());
        telemetry.addData("Arm Extension Position", armExtensionMotor.getCurrentPosition());
      //  telemetry.addData("Hanging Arm Position", hangingArm.getCurrentPosition());
        telemetry.addData("Max extension", maxExtension);

        // Send telemetry data to driver station
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addData("Speed Mode", speedScale);
        telemetry.addData("Heading", Math.toDegrees(botHeadingRadians));
        telemetry.update();
    }
}
