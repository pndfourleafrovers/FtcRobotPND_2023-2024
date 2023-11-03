package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="MoveRobot", group="Autonomous")
@Disabled
public class MoveRobot extends LinearOpMode {

    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private DistanceSensor sensorLeft;
    private DistanceSensor sensorRight;
    private static final double DETECTION_THRESHOLD = 20.0;  // e.g., 20 cm
    boolean objectDetectedLeft = false;
    boolean objectDetectedRight = false;
    boolean lookForProp = false;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    private IMU imu = null;      // Control/Expansion Hub IMU

    @Override
    public void runOpMode() throws InterruptedException {
        // Step 1: Initialization
        initHardware();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        // Wait for the game to start (driver presses PLAY)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();
        turnToHeading(0);
        driveForward(24, 0.5);
        lookForProp = true;
        driveForward(6,0.5);
        lookForProp = false;

        if (objectDetectedLeft) {
            turnToHeading(90);
            driveForward(5,0.5);

            driveBackward(5,0.8);
            turnToHeading(0);

            driveForward(25,0.8);
            turnToHeading(-90);

            driveForward(65,1);
            turnToHeading(-135);
        } else if (objectDetectedRight) {
            turnToHeading(-90);
            driveForward(5,0.5);

            driveBackward(5,.8);
            turnToHeading(0);

            driveForward(25,.8);
            turnToHeading(-90);

            driveForward(65,1);
            turnToHeading(-135);
        } else {
            // Continue forward if no objects were detected
            driveForward(5, 0.5);

            driveBackward(5,.8);
            turnToHeading(90);

            driveForward(20,.8);
            turnToHeading(0);

            driveForward(25,.8);
            turnToHeading(-90);

            driveForward(85,1);
            turnToHeading(-135);
        }


    }

    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "Right_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "Left_rear");
        rightBackDrive = hardwareMap.get(DcMotor.class, "Right_rear");
        sensorLeft = hardwareMap.get(DistanceSensor.class, "LeftD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.
        sensorRight = hardwareMap.get(DistanceSensor.class, "RightD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void driveBackward(double distance, double power) {
        driveForward(-distance, power);
    }
    public void driveForward(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (520 / (2.25*1.5/2 * Math.PI)); //520 encoder ticks per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);
        // Set the target positions for each motor
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticksToMove);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + ticksToMove);

        // Set the mode to RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
            if (lookForProp){
                if (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
                    objectDetectedLeft = true;
                    break;  // Exit the loop if we detect an object
                }
                if (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD) {
                    objectDetectedRight = true;
                    break;  // Exit the loop if we detect an object
                }



            }

        }


        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void turnToHeading(int targetHeading) {
        double turnKp = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getHeading() - targetHeading) > 1) { // Allow for a small error margin

            double error = targetHeading - getHeading(); // Calculate the error

            // Calculate wheel powers based on error
            double turnPower = error * turnKp;

            double leftFrontPower    =  -turnPower;
            double rightFrontPower   =  +turnPower;
            double leftBackPower     =  -turnPower;
            double rightBackPower    =  +turnPower;

            // Ensure wheel powers do not exceed 1.0
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();

        }
        // Stop all the motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }
}