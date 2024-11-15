package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive;

@TeleOp
public class ITDMain extends OpMode
{   Servo armclaw;
    DcMotor armRotationMotor, armExtensionMotor, frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotorSimple hangingArm;
    IMU imu;
    boolean armExtensionOutOfBounds = false;
    double robot_speed = 0.5;


    @Override
    public void init()
    {
        //DRIVE
        frontLeftMotor = hardwareMap.dcMotor.get("LF");
        backLeftMotor = hardwareMap.dcMotor.get("LR");
        frontRightMotor = hardwareMap.dcMotor.get("RF");
        backRightMotor = hardwareMap.dcMotor.get("RR");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        //CLAW
        armclaw = hardwareMap.get(Servo.class, "claw_grab");
        armclaw.scaleRange(-0.1, 0.1);
        // ARM
        armRotationMotor = hardwareMap.get(DcMotor.class, "arm_rotation");
        armRotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtensionMotor = hardwareMap.get(DcMotor.class, "arm_extension");
        armExtensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //HANGING
        hangingArm = hardwareMap.get(DcMotorSimple.class, "hanging");

    }

    @Override
    public void loop()
    {
        Telemetry();
        Drive();
        Debug();
        //ARM ROTATION
        //arm goes down
        if (gamepad2.dpad_down)
        {
            while (armRotationMotor.getCurrentPosition() < 1100){
                armRotationMotor.setTargetPosition(1100);
                armRotationMotor.setPower(0.5);
                armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //DOWN HAS POWER NEGATIVE
        }
        //arm goes up
        else if (gamepad2.dpad_up) {
            while (armRotationMotor.getCurrentPosition() > 20){
                armRotationMotor.setTargetPosition(20);
                armRotationMotor.setPower(-0.6);
                armRotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        armRotationMotor.setTargetPosition(Math.max(Math.min((armRotationMotor.getCurrentPosition() - (int)(gamepad2.left_stick_y * 10)), 1300),20));
        armExtensionMotor.setTargetPosition(Math.max(Math.min((armExtensionMotor.getCurrentPosition() - (int)(gamepad2.right_stick_y * 10)), 1100),0));

        //HANGING

        if (gamepad2.right_bumper){hangingArm.setPower(1.0);}
        else if (gamepad2.left_bumper) {hangingArm.setPower(-1.0);}
        else {hangingArm.setPower(0.0);}



        // CLAW
        if (gamepad2.x) armclaw.setPosition(0.0);
        if (gamepad2.b) armclaw.setPosition(0.3);

    }
    void Drive()
    {
        double y = -gamepad1.left_stick_y * robot_speed;
        double x = gamepad1.left_stick_x * robot_speed;
        double rx = gamepad1.right_stick_x;


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(rotY) + abs(rotX) + abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    void Telemetry()
    {
        //telemetry.addData("Claw servo position", armclaw.getPosition());
        telemetry.addData("Arm rotation ticks", armRotationMotor.getCurrentPosition());
        telemetry.addData("Arm extension ticks", armExtensionMotor.getCurrentPosition());
        telemetry.addData("arm extension out of bounds", armExtensionOutOfBounds);
    }
    void Debug(){
        if (gamepad1.dpad_right) armRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (gamepad1.dpad_left) armExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (gamepad1.back) imu.resetYaw();
    }
}
