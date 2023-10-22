package Old.RedFrontQ1.RedBackQ4;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Mixed.AprilAuto;
import org.firstinspires.ftc.teamcode.Objects.Odometry;

import org.firstinspires.ftc.teamcode.Mixed.AprilAuto;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name="RedCenterRight", group="Autonomous")
public class RedCenterRight extends LinearOpMode{
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor RearLeft;
    private DcMotor RearRight;
    private IMU imu;      // Control/Expansion Hub IMU

    private double headingError = 0;

    private double targetHeading = 0;
    private double driveSpeed = 0;

    //not sure if most of these variables are needed. Might get rid of them to see at some point.
    private double  FrontLeftSpeed     = 0;
    private double  FrontRightSpeed    = 0;
    private double  RearLeftSpeed     = 0;
    private double  RearRightSpeed    = 0;
    private int     FrontLeftTarget    = 0;
    private int     FrontRightTarget   = 0;
    private int     RearLeftTarget    = 0;
    private int     RearRightTarget   = 0;

    static final double TICKS_PER_MOTOR_REV = 520.0;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.5;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 2.5;     // For figuring circumference
    static final double COUNTS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double DRIVE_SPEED = 0.5;     // Max driving speed for better distance accuracy.
    static final double TURN_SPEED = 0.3;     // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 0.5;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    private double  turnSpeed     = 0;
    private ColorSensor leftColor, rightColor;

    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

    private int DESIRED_TAG_ID;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    int Position; //(+3 if red)
    @Override
    public void runOpMode() {


        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        initAprilTag();

        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        RearRight.setDirection(DcMotor.Direction.REVERSE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review
        //   ambient();?
        driveStraight(DRIVE_SPEED, 36.0, 0.0);    // In inches
        turnToHeading(TURN_SPEED, 0.0);               // In Degrees
        holdHeading(TURN_SPEED, 0.0, 0.25);
//Have prop detection code here

        ambient();
        prop();

//Place pixel on prop spot, grab code

        driveStraight(DRIVE_SPEED, 48.0, -90.0);    // In inches
        turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        holdHeading(TURN_SPEED, -90, 0.25);

        driveStraight(DRIVE_SPEED, 18.0, -180.0);    // In inches
        turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        holdHeading(TURN_SPEED, -90, 0.25);




//April tag detection and drive to.
        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            DESIRED_TAG_ID = Position + 3; // = whatever we decided to name the variable at the beginning
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","No valid target\n");
            }

            // If we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveApril(drive, strafe, turn);
            sleep(10);
            break;
        }


//Correct position angle
        turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        holdHeading(TURN_SPEED, -90, 0.25);

//Place pixel on correct spot
// Go to park
        redRight();

    }


    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            FrontLeftTarget = FrontLeft.getCurrentPosition() + moveCounts;
            FrontRightTarget = FrontRight.getCurrentPosition() + moveCounts;
            RearLeftTarget = FrontLeft.getCurrentPosition() + moveCounts;
            RearRightTarget = FrontRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FrontLeft.setTargetPosition(FrontLeftTarget);
            FrontRight.setTargetPosition(FrontRightTarget);
            RearLeft.setTargetPosition(RearLeftTarget);
            RearRight.setTargetPosition(RearRightTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && RearLeft.isBusy() && RearRight.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        FrontLeftSpeed  = drive - turn;
        FrontRightSpeed = drive + turn;
        RearLeftSpeed  = drive - turn;
        RearRightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max1 = Math.max(Math.abs(FrontLeftSpeed), Math.abs(FrontRightSpeed));
        double max2 = Math.max(Math.abs(RearRightSpeed), Math.abs(RearLeftSpeed));
        if (max1 > 1.0 || max2 > 1.0)
        {
            FrontLeftSpeed /= max1;
            FrontRightSpeed /= max1;
            RearLeftSpeed /= max2;
            RearRightSpeed /= max2;
        }

        FrontLeft.setPower(FrontLeftSpeed);
        FrontRight.setPower(FrontRightSpeed);
        RearLeft.setPower(RearLeftSpeed);
        RearRight.setPower(RearRightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      FrontLeftTarget,  FrontRightTarget, RearLeftTarget,  RearRightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      FrontLeft.getCurrentPosition(),
                    FrontRight.getCurrentPosition(), RearLeft.getCurrentPosition(), RearRight.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", FrontLeftSpeed, FrontRightSpeed, RearRightSpeed, RearLeftSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void moveApril(double x, double y, double yaw) {
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
        FrontLeft.setPower(leftFrontPower);
        FrontRight.setPower(rightFrontPower);
        RearLeft.setPower(leftBackPower);
        RearRight.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    public void initAprilTag() {
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    public void    setManualExposure(int exposureMS, int gain) {
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
    public void ambient () {
        int leftAmbient = leftColor.alpha();
        int rightAmbient = rightColor.alpha();

        if (leftAmbient > 75) { // Adjust the value based on your observations and testing
            Position = 1;
        } else if (rightAmbient > 120) { // Adjust the value based on your observations and testing
            Position = 2;
        } else if (leftAmbient < 75 && rightAmbient < 120) {
            Position = 3;
        }
    }
    public void prop (){
//Make holdTime a variable
        switch (Position) {
            case 1:
                driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                turnToHeading(TURN_SPEED, 30);               // In Degrees
                holdHeading(TURN_SPEED, 30, 0.25);

                driveStraight(DRIVE_SPEED, 10, 30);    // In inches
                turnToHeading(TURN_SPEED, 30);               // In Degrees
                holdHeading(TURN_SPEED, 0, 0.25);
                //Grab code
                driveStraight(DRIVE_SPEED, -10, 30);    // In inches
                turnToHeading(TURN_SPEED, 0);               // In Degrees
                holdHeading(TURN_SPEED, 0, 0.25);

                driveStraight(DRIVE_SPEED, 40, 0);    // In inches
                turnToHeading(TURN_SPEED, 0);               // In Degrees
                holdHeading(TURN_SPEED, 0, 0.25);
                break;
            case 2:
                driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                turnToHeading(TURN_SPEED, -30);               // In Degrees
                holdHeading(TURN_SPEED, -30, 0.25);

                driveStraight(DRIVE_SPEED, 10, -30);    // In inches
                turnToHeading(TURN_SPEED, -30);               // In Degrees
                holdHeading(TURN_SPEED, -30, 0.25);
                // Grab code
                driveStraight(DRIVE_SPEED, -10, -30);    // In inches
                turnToHeading(TURN_SPEED, 0);               // In Degrees
                holdHeading(TURN_SPEED, 0, 0.25);

                driveStraight(DRIVE_SPEED, 40, 0);    // In inches
                turnToHeading(TURN_SPEED, 0);               // In Degrees
                holdHeading(TURN_SPEED, 0, 0.25);
                break;
            case 3:
                driveStraight(DRIVE_SPEED, -20, 0.0);    // In inches
                turnToHeading(TURN_SPEED, 10);               // In Degrees
                holdHeading(TURN_SPEED, 10, 0.25);

                driveStraight(DRIVE_SPEED, 30, 10);    // In inches
                turnToHeading(TURN_SPEED, 10);               // In Degrees
                holdHeading(TURN_SPEED, 10, 0.25);
                //Grab code
                driveStraight(DRIVE_SPEED, -30, 30);    // In inches
                turnToHeading(TURN_SPEED, 90);               // In Degrees
                holdHeading(TURN_SPEED, 90, 0.25);

                driveStraight(DRIVE_SPEED, 25, 90);    // In inches
                turnToHeading(TURN_SPEED, 0);               // In Degrees
                holdHeading(TURN_SPEED, 0, 0.25);

                driveStraight(DRIVE_SPEED, 40, 0);    // In inches
                turnToHeading(TURN_SPEED, -90);               // In Degrees
                holdHeading(TURN_SPEED, -90, 0.25);

                driveStraight(DRIVE_SPEED, 25, -90);    // In inches
                turnToHeading(TURN_SPEED, -90);               // In Degrees
                holdHeading(TURN_SPEED, -90, 0.25);
                break;
            //By the end of each sequence, each movement is in same proximate position somewhere above the middle spike.
        }
    }
    public void redRight (){
        driveStraight(DRIVE_SPEED, 18.0, -180.0);    // In inches
        turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        holdHeading(TURN_SPEED, -90, 0.25);

        driveStraight(DRIVE_SPEED, 18.0, -180.0);    // In inches
        turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        holdHeading(TURN_SPEED, -90, 0.25);

        driveStraight(DRIVE_SPEED, 18.0, -180.0);    // In inches
        turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        holdHeading(TURN_SPEED, -90, 0.25);
    }
}
