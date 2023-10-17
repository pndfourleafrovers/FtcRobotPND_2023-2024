package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Poop", group="Autonomous")
public class Poop extends LinearOpMode {
    private IMU imu;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private ColorSensor leftColor, rightColor;

    private static final double TICKS_PER_REV = 537;
    private static final double WHEEL_DIAMETER = 5;
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
    private static final double WHEELBASE_WIDTH = 13;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        driveAndDetect();


    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_rear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");

        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    private void driveAndDetect() {
        driveForwardUsingEncoder(60);


        resetEncoders();
        boolean detectedLeftOrRight = false;
        int targetTicksFor12Inches = (int) (12 * TICKS_PER_INCH);

        while(Math.abs(leftFrontDrive.getCurrentPosition()) < targetTicksFor12Inches && opModeIsActive()) {
            leftFrontDrive.setPower(.2);
            rightFrontDrive.setPower(.2);
            leftBackDrive.setPower(.2);
            rightBackDrive.setPower(.2);


            String detected = findPropOnLeftorRight();

            if(detected.equals("left")) {
                detectedLeftOrRight = true;
                telemetry.addData("Detection:", "Object detected on the LEFT!");
                turnUsingEncoder(-90);
                break;
            } else if(detected.equals("right")) {
                detectedLeftOrRight = true;
                telemetry.addData("Detection:", "Object detected on the RIGHT!");
                turnUsingEncoder(90);
                break;
            }

        }

        stopMotors();

        // If nothing was detected on the left or right during the 12-inch drive
        if (!detectedLeftOrRight) {
            // Assume the object is in front and act accordingly.
            // You can add any action you want here, such as driving forward a bit more or something else
            telemetry.addData("Detection:", "DRIVING FORWARD!");
            driveForwardUsingEncoder(46);
            telemetry.update(); // Update telemetry data on the driver station
        }
    }


    private void driveForwardUsingEncoder(double inches) {
        // Assuming the robot moves straight for the given inches.
        int targetTicks = (int) (inches * TICKS_PER_INCH);
        resetEncoders();

        while(Math.abs(leftFrontDrive.getCurrentPosition()) < targetTicks && opModeIsActive()) {
            leftFrontDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
        }
        stopMotors();
    }

    private String findPropOnLeftorRight() {
        int leftAmbient = leftColor.alpha();
        int rightAmbient = rightColor.alpha();

        if (leftAmbient > 75) { // Adjust the value based on your observations and testing
            return "left";
        }
        if (rightAmbient > 120) { // Adjust the value based on your observations and testing
            return "right";
        }
        return "none";
    }

    private void turnUsingEncoder(double degrees) {
        double turningDistance = (WHEELBASE_WIDTH * Math.PI * degrees) / 360.0; // Distance one side of the robot should travel
        int targetTicks = (int) (turningDistance * TICKS_PER_INCH);

        resetEncoders();

        if (degrees > 0) { // turning right
            while (Math.abs(leftFrontDrive.getCurrentPosition()) < targetTicks && opModeIsActive()) {
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(-0.5);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(-0.5);
            }
        } else { // turning left
            while (Math.abs(rightFrontDrive.getCurrentPosition()) < targetTicks && opModeIsActive()) {
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);
            }
        }

        stopMotors();
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void resetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

}

