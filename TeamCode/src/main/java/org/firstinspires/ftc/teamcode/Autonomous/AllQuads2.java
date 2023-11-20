package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Components.AprilTagFinder;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Drivetrain;
import org.firstinspires.ftc.teamcode.Components.QuadrantPot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.Components.Arm;
@Autonomous(name="AllQuads2", group="Autonomous")
//@Disabled
public class AllQuads2 extends LinearOpMode {
    //components
    final Arm m_Arm = new Arm();
    final Drivetrain m_Drivetrain = new Drivetrain();
    public com.qualcomm.robotcore.hardware.DistanceSensor sensorLeft;
    public com.qualcomm.robotcore.hardware.DistanceSensor sensorRight;
    final QuadrantPot m_QuadrantPot = new QuadrantPot();
    final Camera m_Camera = new Camera();
    final double DESIRED_DISTANCE = 13.5; //  this is how close the camera should get to the target (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    public final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    public final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public final double DETECTION_THRESHOLD = 20.0;  // e.g., 20 cm this is used for detecting the prop
    public static final double DETECTION_THRESHOLD2 = 12.7;  // e.g., 12.7 cm thi is used to see if there is another robot in the path while strafing
    public boolean objectDetectedLeft = false; // this is for looking for the prop on spike mark
    boolean detectProp = true; // this says we have detected the prop and are performing actions related to having found the prop
    public boolean objectDetectedRight = false;// this is for looking for the prop on spike mark
    protected boolean lookForProp = false; // this tells the robot to look for the prop when true, while driving forward
    public boolean lookForRobot =false;
    boolean atBackdrop = false;
    boolean ENDGAME = false;
    TouchSensor touchSensor;  // Touch sensor Object
    boolean startOfAutonomous = true;
    //These are for setting the speed of the robot
    double fwdPwr = 0.6; //how fast to mover forward
    double bwdPwr = 0.6; //how fast to move backward
    double strafePwr= 0.6; //how fast to strafe
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectedTag = null;     // Used to hold the data for a detected AprilTag
    boolean findAprilTag = false;
    int Position;
    private Servo pmmA;
    private Servo pmmF;
    static final int TICKS_PER_MOTOR_REV = 1425; //both servos pmmA and pmmF have the same resolution
    public static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120; // there is a gear that have a 3 to 1 ratio
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    private IMU imu = null;      // Control/Expansion Hub IMU

    @Override
    public void runOpMode() {
        // Step 1: Initialization
        initHardware();
        int quadrant = m_QuadrantPot.getPotentiometerQuadrant();
        boolean targetFound = false;    // APRIL TAG: Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)
        initAprilTag();// APRIL TAG:
        AprilTagFinder tagSearcher = new AprilTagFinder(aprilTag, telemetry);


        if (USE_WEBCAM) // APRIL TAG:
            m_Camera.setManualExposure(6, 250);
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            // when the Robot is at the backdrop and aligned to the correct April Tag
            while (atBackdrop) {
                //We are at the correct location relative to the April Tag, stop the motors
                m_Drivetrain.SetPower(0);
                //Here we are moving slightly to the left or right of the april tag to avoid that the pixel bounces out of place
                if (DESIRED_TAG_ID==6 || DESIRED_TAG_ID ==3){
                    m_Drivetrain.strafeRight(2);
                } else if (DESIRED_TAG_ID==4 || DESIRED_TAG_ID ==1){
                    m_Drivetrain.strafeLeft(2);
                }
                //sometimes the april tag code does not align properly, let the IMU do that for you
                m_Drivetrain.turnToHeading((quadrant==1||quadrant==2)?-90:90);
                //Position the arm to drop the pixel
                m_Arm.moveArmToBackDrop();
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
                m_Arm.resetArm();
                // pressing the touch sensor goes to center, not pressing the touch sensor goes to the corners
                m_Drivetrain.parkRobot(quadrant, Position, touchSensor.isPressed()); //we want to park the robot based on position, quadrant and if the touchsensor is pressed
                 //final drive before parking in the backstage
                m_Drivetrain.driveForward(10);
                m_Arm.dropArmToZeroAndReset();
                requestOpModeStop();
            }
            //THIS IS THE START OF AUTONOMOUS
            if (startOfAutonomous) {
                //The robot is at the wall at the start of Autonomous. Remember that the direction of the IMU sets the 0 and the start of each run.
                m_Arm.HoldArmOffGround();
                //Set the position of the Arm Pixel Holder or PMMA
                pmmA.setDirection(Servo.Direction.FORWARD);
                pmmA.setPosition(0.55);
                //Set the position of the floor Pixel Holder of PMMF
                pmmF.setDirection(Servo.Direction.REVERSE);
                pmmF.setPosition(0.62);
                //start driving towards the prop
                m_Drivetrain.turnToHeading(0);
                //initially the robot has the robot facing towards the wall so we have to move the robot backwards
                //we want to get past the truss before we start looking for the prop
                m_Drivetrain.driveBackward(21);
                //We have passed the truss start looking for the prop
                lookForProp = true;
                //drive backwards until you find where the prop is located, the distance sensors detect left and right and if neither then it is front
                m_Drivetrain.driveBackward(11);
                //stop looking for the prop
                lookForProp = false;
                m_Drivetrain.driveForward(6);
                startOfAutonomous = false;
            }
            //Detecting the prop on the spike mark

            if (detectProp) {
                if (objectDetectedLeft && quadrant == 3 || objectDetectedRight && quadrant == 2) {
                    //Position sets which April Tag to look for i.e. 1 for left if on the blue side, 3 is added if the robot is on the red side
                    // In quadrant 2 we have to switch the position because the robot was turned and left is now near the back drop, in quadrant 2 left was at the truss
                    Position = (quadrant == 3) ? 1 : 3;
                    //This sets how far the robot strafes during parking
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 2) ? -1 : 1));
                    m_Drivetrain.driveBackward(3.5);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    m_Drivetrain.driveForward(5);
                    //April detections after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (objectDetectedRight && quadrant == 3 || objectDetectedLeft && quadrant == 2) {
                    //Position sets which April Tag to look for i.e. 3 for right if on the blue side, 3 is added if the robot is on the red side
                    // In quadrant 2 we have to switch the position because the robot was turned and left is now near the back drop, in quadrant 2 left was at the truss
                    Position = (quadrant == 3) ? 3 : 1;//This also sets how far the robot strafes during parking
                    m_Drivetrain.turnToHeading(-90 * ((quadrant == 2) ? -1 : 1)); //if you are in quadrant 2 change to +90 degrees
                    m_Drivetrain.driveBackward(3.5);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);
                    m_Drivetrain.driveForward(3.5);

                    if (quadrant == 3) {
                        m_Drivetrain.strafeLeft(23);
                    } else {
                        m_Drivetrain.strafeRight(23);
                    }
                    m_Drivetrain.driveBackward(25);
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 2) ? -1 : 1));
                    if (quadrant == 3) {
                        m_Drivetrain.strafeLeft(15);
                    } else {
                        m_Drivetrain.strafeRight(15);
                    }
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (quadrant == 3 || quadrant == 2) {
                    //Position sets which April Tag to look for i.e. 3 for center if on the blue side, 3 is added if the robot is on the red side
                    Position = 2;
                    // Continue forward if no objects were detected
                    m_Drivetrain.driveBackward(4.5);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    m_Drivetrain.driveForward(5.5);
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 2) ? -1 : 1));   //if you are in quadrant 2 change to -90 degrees
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (objectDetectedLeft && quadrant == 4 || objectDetectedRight && quadrant == 1) {
                    Position = (quadrant == 4) ? 1 : 3;
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 1) ? -1 : 1));
                    m_Drivetrain.driveBackward(4.5);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    m_Drivetrain.driveForward(4);
                    if (quadrant == 4) {
                        m_Drivetrain.strafeLeft(29);
                    } else {
                        m_Drivetrain.strafeRight(29);
                    }
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 1) ? -1 : 1));
                    m_Drivetrain.driveForward(70);
                    if (quadrant == 4) {
                        lookForRobot=true;
                        m_Drivetrain.strafeRight(21);
                    } else {
                        lookForRobot=true;
                        m_Drivetrain.strafeLeft(21);
                    }
                    //April detections after this
                    findAprilTag = true;
                    detectProp = false;

                } else if (objectDetectedRight && quadrant == 4 || objectDetectedLeft && quadrant == 1) {
                    Position = (quadrant == 4) ? 3 : 1;
                    m_Drivetrain.turnToHeading(-90 * ((quadrant == 1) ? -1 : 1));
                    m_Drivetrain.driveBackward(3.5);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    m_Drivetrain.driveForward(4);
                    if (quadrant == 4) {
                        m_Drivetrain.strafeRight(28);
                    } else {
                        m_Drivetrain.strafeLeft(28);
                    }
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 1) ? -1 : 1));
                    m_Drivetrain.driveForward(72);
                    if (quadrant == 4) {
                        lookForRobot=true;
                        m_Drivetrain.strafeRight(32);
                    } else {
                        lookForRobot=true;
                        m_Drivetrain.strafeLeft(32);
                    }
                    //April detection after this
                    findAprilTag = true;
                    detectProp = false;
                } else if (quadrant == 4 || quadrant == 1) {
                    Position = 2;
                    // Continue forward if no objects were detected
                    m_Drivetrain.driveBackward(4.5);

                    //Release the pixel on the floor PMM on the spike line
                    pmmF.setDirection(Servo.Direction.REVERSE);
                    pmmF.setPosition(0);

                    m_Drivetrain.driveForward(4.5);
                    if (quadrant == 4) {
                        m_Drivetrain.strafeRight(15);
                    } else {
                        m_Drivetrain.strafeLeft(15);
                    }
                    m_Drivetrain.driveBackward(27);
                    m_Drivetrain.turnToHeading(90 * ((quadrant == 1) ? -1 : 1));
                    m_Drivetrain.driveForward(85);
                    if (quadrant == 4) {
                        lookForRobot=true;
                        m_Drivetrain.strafeRight(21);
                    } else {
                        lookForRobot=true;
                        m_Drivetrain.strafeLeft(21);
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
                m_Drivetrain.moveAprilRobot(drive, strafe, turn);

                if (detectedTag != null && (detectedTag.ftcPose.range - DESIRED_DISTANCE) <= 0.05) {
                    atBackdrop = true;
                    findAprilTag = false;
                }

            }
        }
    }

     private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        m_Drivetrain.InitDrivetrain();
        m_QuadrantPot.InitQuadrantPot();
        m_Arm.initServos();
        m_Arm.initArm();
        // only implement this code if you have the touch sensor driving where to park
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        sensorLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "LeftD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.
        sensorRight = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, "RightD"); // Replace 'sensorLeftName' with the name you've given the sensor in the configuration.

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
}
