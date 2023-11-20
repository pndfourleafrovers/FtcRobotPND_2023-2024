package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Random.Mixed.AprilTagFinder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/AllQuadsFinal.java
@Autonomous(name="AllQuadsFinal", group="Autonomous")
//@Disabled
public class AllQuadsFinal extends LinearOpMode {
========
@Autonomous(name="NewAllQuads", group="Autonomous")
//@Disabled
public class NewAllQuads extends LinearOpMode {
>>>>>>>> origin/master:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous/NewAllQuads.java
    final double DESIRED_DISTANCE = 13.5; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
    private DistanceSensor sensorLeft;
    private DistanceSensor sensorRight;
    private static final double DETECTION_THRESHOLD = 20.0;  // e.g., 20 cm
    private static final double DETECTION_THRESHOLD2 = 12.7;  // e.g., 20 cm
    boolean objectDetectedLeft = false;
    boolean detectProp = true;
    boolean objectDetectedRight = false;
    boolean lookForProp = false;
    boolean WATCHOUT = true;
    boolean atBackdrop = false;
    boolean ENDGAME = false;
    boolean startOfAutonomous = true;
    //These are for setting the speed of the robot
    double fwdPwr = 0.6;
    double bwdPwr = 0.6;
    double strafePwr= 0.6;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectedTag = null;     // Used to hold the data for a detected AprilTag
    boolean findAprilTag = false;

    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    int Position;
    int count = 0;
    double remainingDistance = 21; // Initialize remaining distance to 21 units

    double[] distToStrafeAtPark ={0, -33, -26, -20};
    private Servo pmmA;
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120;
    final double TICKS_PER_DEGREES = 12;  // Adjust this value as necessary

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    private Servo pmmF;
    private IMU imu = null;      // Control/Expansion Hub IMU
    AnalogInput potentiometer;
    //This part is to create a state machine that keeps track of the state of the robot and the time at which actions are taken.

    DcMotor arm;

    enum State {
        MOVING,
        WAITING,
        READY_TO_MOVE
    }

    State state = State.READY_TO_MOVE;
    ElapsedTime timer = new ElapsedTime();
    final double WAIT_TIME = 1.0;       // Time to wait in seconds
    int moveArmToPosition = 7;          // Position to move the arm to in degrees

    @Override
    public void runOpMode() {
        // Step 1: Initialization
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        int quadrant = getPotentiometerQuadrant();
        boolean targetFound = false;    // APRIL TAG: Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        pmmF = hardwareMap.get(Servo.class, "pmmfloor");
        pmmA = hardwareMap.get(Servo.class, "pmmA");
        arm = hardwareMap.get(DcMotor.class, "arm");
        //distance to strafe a park is how far to move during autonomous to park the robot
        //to make it easier, the first number is just a placeholder
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        initAprilTag();// APRIL TAG:
        AprilTagFinder tagSearcher = new AprilTagFinder(aprilTag, telemetry);
        initHardware();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        // Wait for the game to start (driver presses PLAY)
        if (USE_WEBCAM) // APRIL TAG:
            setManualExposure(6, 250);
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            // when the Robot is at the backdrop and aligned to the correct April Tag
            while (atBackdrop) {
                //We are at the correct location relative to the April Tag, stop the motors
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                //Here we are moving slightly to the left or right of the april tag to avoid that the pixel bounces out of place
                if (DESIRED_TAG_ID==6 || DESIRED_TAG_ID ==3){
                    strafeRight(2,strafePwr);
                } else if (DESIRED_TAG_ID==4 || DESIRED_TAG_ID ==1){
                    strafeLeft(2,strafePwr);
                }
                //Position the arm to drop the pixel
                arm.setTargetPosition(TICKS_PER_DEGREE * 210);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                //give the arm time to get to the backdrop
                sleep(2000);
                //release the pixel
                pmmA.setPosition(0);
                //Moving away from the backdrop
                atBackdrop = false;
                //Park the robot
                ENDGAME = true;
            }
            //Proceed to Parking the robot
            if (ENDGAME){
                //Give the arm time to get back to the position just off of the floor
                sleep(1000);
                arm.setTargetPosition(TICKS_PER_DEGREE * 7);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.1);
                sleep(2000);
                //This is where to park if we are in quadrant 1 i.e. front of the blue then park in the middle
                if (quadrant==1){
                    strafeRight(distToStrafeAtPark[Position], strafePwr);
                } else if (quadrant==2) { //park in the corner, since the travel from 3 is the longest assign it 1 in the array distToStrafeAtPark because it is the largest number
                    strafeLeft(distToStrafeAtPark[(Position==1)?3:(Position==3)?1:Position],strafePwr);
                } else if (quadrant==3) {
                    strafeRight(distToStrafeAtPark[Position], strafePwr);
                } else {
                    strafeLeft(distToStrafeAtPark[(Position==1)?3:(Position==3)?1:Position],strafePwr);
                }
                //final drive before parking in the backstage
                driveForward(10,fwdPwr);
                //arm.setTargetPosition(TICKS_PER_DEGREE * 0);
                //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //arm.setPower(0.1);
                //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                requestOpModeStop();
            }
            //THIS IS THE START OF AUTONOMOUS
            if (startOfAutonomous) {
                //The robot is at the wall at the start of Autonomous. Remember that the direction of the IMU sets the 0 and the start of each run.
                // Lift the ARM 7 degrees off of the ground and hold it there
                arm.setTargetPosition(TICKS_PER_DEGREE * 7);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                //Set the position of the Arm Pixel Holder or PMMA
                pmmA.setDirection(Servo.Direction.FORWARD);
                pmmA.setPosition(0.55);
                //Set the position of the floor Pixel Holder of PMMF
                pmmF.setDirection(Servo.Direction.REVERSE);
                pmmF.setPosition(0.62);
                //start driving towards the prop
                turnToHeading(0);
                //initially the robot has the robot facing towards the wall so we have to move the robot backwards
                //we want to get past the truss before we start looking for the prop
                driveBackward(21, bwdPwr);
                //We have passed the truss start looking for the prop
                lookForProp = true;
                //drive backwards until you find where the prop is located, the distance sensors detect left and right and if neither then it is front
                driveBackward(11, bwdPwr);
                //stop looking for the prop
                lookForProp = false;
                driveForward(6, 0.5);
                startOfAutonomous = false;
            }
            //Detecting the prop on the spike mark

            if (detectProp) {
                if (objectDetectedLeft&&quadrant==3||objectDetectedRight&&quadrant==2) {
                    //Position sets which April Tag to look for i.e. 1 for left if on the blue side, 3 is added if the robot is on the red side
                    // In quadrant 2 we have to switch the position because the robot was turned and left is now near the back drop, in quadrant 2 left was at the truss
                    Position = (quadrant==3)?1:3;
                    //This sets how far the robot strafes during parking
                    turnToHeading(90*((quadrant==2)?-1:1));
                    driveBackward(3.5, bwdPwr);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    driveForward(5, fwdPwr);
                    //April detections after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (objectDetectedRight&&quadrant==3||objectDetectedLeft&&quadrant==2) {
                    //Position sets which April Tag to look for i.e. 3 for right if on the blue side, 3 is added if the robot is on the red side
                    // In quadrant 2 we have to switch the position because the robot was turned and left is now near the back drop, in quadrant 2 left was at the truss
                    Position = (quadrant==3)?3:1;//This also sets how far the robot strafes during parking
                    turnToHeading(-90*((quadrant==2)?-1:1)); //if you are in quadrant 2 change to +90 degrees
                    driveBackward(3.5, bwdPwr);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);
                    driveForward(3.5,fwdPwr);

                    if (quadrant==3){
                        strafeLeft(23,strafePwr);
                    } else {
                        strafeRight(23,strafePwr);
                    }
                    driveBackward(25,bwdPwr);
                    turnToHeading(90*((quadrant==2)?-1:1));
                    if (quadrant==3){
                        strafeLeft(15,strafePwr);
                    } else {
                        strafeRight(15,strafePwr);
                    }
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (quadrant==3||quadrant==2) {
                    //Position sets which April Tag to look for i.e. 3 for center if on the blue side, 3 is added if the robot is on the red side
                    Position = 2;
                    // Continue forward if no objects were detected
                    driveBackward(4.5, bwdPwr);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    driveForward(5.5, .5);
                    turnToHeading(90*((quadrant==2)?-1:1));   //if you are in quadrant 2 change to -90 degrees
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (objectDetectedLeft&&quadrant==4||objectDetectedRight&&quadrant==1){
                    Position = (quadrant==4)?1:3;
                    turnToHeading(90*((quadrant==1)?-1:1));
                    driveBackward(4.5, bwdPwr);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    driveForward(4, fwdPwr);
                    if(quadrant==4){
                        strafeLeft(29,strafePwr);
                    } else {
                        strafeRight(29,strafePwr);
                    }
                    turnToHeading(90*((quadrant==1)?-1:1));
                    driveForward(70,1);
                    WATCHOUT = true;
                    if (WATCHOUT){
                        if(quadrant==4){
                            strafeRightWithObstacleDetection(21, strafePwr, DETECTION_THRESHOLD2);
                        } else {
                            strafeLeftWithObstacleDetection(21, strafePwr, DETECTION_THRESHOLD2);
                        }
                        WATCHOUT = false;
                    }

                    //April detections after this
                    findAprilTag = true;
                    detectProp = false;

                } else if (objectDetectedRight&&quadrant==4||objectDetectedLeft&&quadrant==1) {
                    Position = (quadrant==4)?3:1;
                    turnToHeading(-90*((quadrant==1)?-1:1));
                    driveBackward(3.5, bwdPwr);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    driveForward(4, fwdPwr);
                    if(quadrant==4){
                        strafeRight(28,strafePwr);
                    } else {
                        strafeLeft(28,strafePwr);
                    }
                    turnToHeading(90*((quadrant==1)?-1:1));
                    driveForward(72,1);
                    if (WATCHOUT){
                        if(quadrant==4){
                            strafeRightWithObstacleDetection(32, strafePwr, DETECTION_THRESHOLD2);
                        } else {
                            strafeLeftWithObstacleDetection(32, strafePwr, DETECTION_THRESHOLD2);
                        }
                        WATCHOUT = false;
                    }
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (quadrant==4||quadrant==1){
                    Position = 2;
                    // Continue forward if no objects were detected
                    driveBackward(4.5, bwdPwr);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    driveForward(4.5, .5);
                    if(quadrant==4){
                        strafeRight(15,strafePwr);
                    } else {
                        strafeLeft(15,strafePwr);
                    }
                    driveBackward(27,0.8);
                    turnToHeading(90*((quadrant==1)?-1:1));
                    driveForward(85,1);
                    if (WATCHOUT){
                        if(quadrant==4){
                            strafeRightWithObstacleDetection(21, strafePwr, DETECTION_THRESHOLD2);
                        } else {
                            strafeLeftWithObstacleDetection(21, strafePwr, DETECTION_THRESHOLD2);
                        }
                        WATCHOUT = false;
                    }
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                }
            }

            while (findAprilTag) {
                //if we are on the blue side, i.e. quadrant 1, or 2, then "Position", set during find prop, is the tag we are looking for
                int quadAdd = 0;  // if the robot is in the blue side find april tags, 1, 2, 3
                //if we are on the red side, i.e. quadrant 3, or 4, then we need to add 3 to "Position", set during find prop, to find the right April Tag
                if (quadrant == 3 || quadrant == 4) {
                    quadAdd = 3; // if the robot is in the red side find april tags 4, 5, 6
                }
                DESIRED_TAG_ID = Position + quadAdd;
                detectedTag = null; // APRIL TAG:
                //The line below creates a instance of the Class tagSearcher which is defined in file AprilTagSearcher
                AprilTagDetection detectedTag = tagSearcher.findTag(DESIRED_TAG_ID);
                // If the AprilTag is detected the robot drives to it
                if (detectedTag != null) {
                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double rangeError = (detectedTag.ftcPose.range - DESIRED_DISTANCE);
                    double headingError = detectedTag.ftcPose.bearing;
                    double yawError = detectedTag.ftcPose.yaw;
                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();
                // Apply desired axes motions to the drivetrain.
                moveAprilRobot(drive, strafe, turn);

                if (detectedTag != null && (detectedTag.ftcPose.range - DESIRED_DISTANCE) <= 0.05) {
                    atBackdrop = true;
                    findAprilTag = false;
                }

            }
        }
    }




    public void moveAprilRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
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
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
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
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void    setManualExposure(int exposureMS, int gain) {
        // THIS IS FOR THE APRIL TAG
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
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
    public int getPotentiometerQuadrant() {
        double voltage = potentiometer.getVoltage();

        if (voltage < 0.7) {
            return 2; // This is the back of the blue alliance
        } else if (voltage >= 0.8 && voltage <= 1.2) {
            return 1; // This is the front of the blue alliance
        } else if (voltage >= 1.4 && voltage <= 2.2) {
            return 4; // This is the front of the red alliance
        } else if (voltage > 2.4) {
            return 3; // This is the back of the red alliance
        } else {
            return 0; // Return 0 if the voltage doesn't match any quadrant, this should only happen if the dial is placed on the black line
        }
    }

    public void strafeLeft(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Positive distance means strafe left, negative means strafe right
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));

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

        // Wait for the motors to finish
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
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
    public void strafeRight(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96/25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);

        // Positive distance means strafe right, negative means strafe left
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));

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

        // Wait for the motors to finish
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
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
    public void strafeRightWithObstacleDetection(double distance, double power, double detectionThreshold) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Set target positions for strafing right
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - ticksToMove);
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

        // Wait for the motors to finish or stop if an obstacle is detected
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Check for obstacle
            if (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                // Obstacle detected, stop all motors
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                // Wait until the obstacle is cleared
                while (sensorLeft.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                    // Optionally add a sleep here to avoid a busy loop
                    sleep(100);
                }

                // Once the obstacle is cleared, resume movement
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(power);
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(power);
            }

            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
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


    public void strafeLeftWithObstacleDetection(double distance, double power, double detectionThreshold) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Set target positions for strafing left
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() - ticksToMove);
        rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + ticksToMove);
        leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + ticksToMove);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() - ticksToMove);

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

        // Wait for the motors to finish or stop if an obstacle is detected
        while (opModeIsActive() && (leftFrontDrive.isBusy() || rightFrontDrive.isBusy() || leftBackDrive.isBusy() || rightBackDrive.isBusy())) {
            // Check for obstacle
            if (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                // Obstacle detected, stop all motors
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);

                // Wait until the obstacle is cleared
                while (sensorRight.getDistance(DistanceUnit.CM) < DETECTION_THRESHOLD2) {
                    // Optionally add a sleep here to avoid a busy loop
                    sleep(100);
                }

                // Once the obstacle is cleared, resume movement
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(power);
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(power);
            }

            // Optionally provide telemetry updates
            telemetry.addData("LeftFrontDrive Position", leftFrontDrive.getCurrentPosition());
            telemetry.addData("RightFrontDrive Position", rightFrontDrive.getCurrentPosition());
            telemetry.addData("LeftBackDrive Position", leftBackDrive.getCurrentPosition());
            telemetry.addData("RightBackDrive Position", rightBackDrive.getCurrentPosition());
            telemetry.update();
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

    private void controlArm(int desiredPosition) {
        switch (state) {
            case READY_TO_MOVE:
                // Ready to move to a new position
                int targetPosition = (int) (TICKS_PER_DEGREES * desiredPosition);
                arm.setTargetPosition(targetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
                state = State.MOVING;
                break;

            case MOVING:
                // Wait for the arm to reach the target position
                if (!arm.isBusy()) {
                    // Arm has reached the target, start the timer and switch to waiting state
                    timer.reset();
                    state = State.WAITING;
                }
                break;

            case WAITING:
                // Wait for the specified time
                if (timer.seconds() > WAIT_TIME) {
                    // Time is up, ready to move to a new position
                    state = State.READY_TO_MOVE;
                }
                break;
        }
    }
}