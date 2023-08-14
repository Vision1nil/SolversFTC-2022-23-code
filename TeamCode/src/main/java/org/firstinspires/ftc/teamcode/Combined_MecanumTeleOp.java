package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Combined_MecanumTeleOp extends LinearOpMode {
    double accelerationFactor = 0.15; // Set the default speed to 15% (0.15).
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    IMU imu;

    boolean isFieldCentric = true; // Default to field-centric mode

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Set the zero power behavior to BRAKE for all motors
        motorFrontLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Field-Centric");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Toggle control mode on left joystick button press
            if (gamepad1.left_stick_button) {
                isFieldCentric = !isFieldCentric; // Toggle the mode
                sleep(200); // Small delay to avoid multiple toggles

                if (isFieldCentric) {
                    telemetry.addData("Mode", "Field-Centric");
                } else {
                    telemetry.addData("Mode", "Robot-Centric");
                }
                telemetry.update();
            }

            if (isFieldCentric) {
                runFieldCentricMode();
            } else {
                runRobotCentricMode();
            }
        }
    }

    private void runFieldCentricMode() {
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x * 1.1;
        double rawRX = gamepad1.right_stick_x;

        double forward = rawY * Math.cos(getBotHeading()) + rawX * Math.sin(getBotHeading());
        double sideways = -rawY * Math.sin(getBotHeading()) + rawX * Math.cos(getBotHeading());
        double rotation = rawRX;

        double lt = gamepad1.left_trigger;
        double speed = accelerationFactor + (1 - accelerationFactor) * lt;

        if (gamepad1.back) {
            imu.resetYaw();
        }

        double frontLeftPower = (forward + sideways + rotation) * speed;
        double backLeftPower = (forward - sideways + rotation) * speed;
        double frontRightPower = (forward - sideways - rotation) * speed;
        double backRightPower = (forward + sideways - rotation) * speed;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        telemetry.addData("Speed", speed);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.update();
    }

    private void runRobotCentricMode() {
// Get raw values from the gamepad
        double y = -gamepad1.left_stick_y; // Negative because the gamepad's y-axis is inverted
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Convert the raw x and y values to robot-centric forward and sideways velocities
        double forward = y * Math.cos(getBotHeading()) + x * Math.sin(getBotHeading());
        double sideways = -y * Math.sin(getBotHeading()) + x * Math.cos(getBotHeading());
        double rotation = rx;

        // Use the LT value as an acceleration factor.
        // LT value is between 0 (not pressed) and 1 (fully pressed).
        double lt = gamepad1.left_trigger;
        double speed = accelerationFactor + (1 - accelerationFactor) * lt;

        // Reset the yaw angle to 0 degrees when the "Back" button is pressed.
        if (gamepad1.back) {
            imu.resetYaw();
        }

        // Calculate motor powers using mecanum drive kinematics
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator * speed;
        double backLeftPower = (rotY - rotX + rx) / denominator * speed;
        double frontRightPower = (rotY - rotX - rx) / denominator * speed;
        double backRightPower = (rotY + rotX - rx) / denominator * speed;

        // Set motor powers
        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);

        telemetry.addData("Speed", speed);
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.update();
    }


    // Helper method to get the robot's heading (yaw) from the IMU
    private double getBotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
}
