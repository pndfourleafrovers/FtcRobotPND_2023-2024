package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Objects.Odometry;

@Disabled
@Autonomous(name="BasicT2", group="Autonomous")
public class BasicTest2 extends LinearOpMode{


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
        @Override
        public void runOpMode() {
            Odometry OdoMethods = new Odometry();
            FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
            RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
            FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
            RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

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

            driveStraight(DRIVE_SPEED, 36.0, 0.0);    // In inches
            turnToHeading(TURN_SPEED, 0.0);               // In Degrees
            holdHeading(TURN_SPEED, 0.0, 1);
//Have prop detection code here
//Place pixel on prop spot
            driveStraight(DRIVE_SPEED, 14.0, 0.0);    // In inches
            turnToHeading(TURN_SPEED, -90.0);               // In Degrees
            holdHeading(TURN_SPEED, -90, 1);

            driveStraight(DRIVE_SPEED, 48.0, -90.0);    // In inches
            turnToHeading(TURN_SPEED, -90.0);               // In Degrees
            holdHeading(TURN_SPEED, -90, 1);

            driveStraight(DRIVE_SPEED, 18.0, -180.0);    // In inches
            turnToHeading(TURN_SPEED, 90.0);               // In Degrees
            holdHeading(TURN_SPEED, 90, 1);
//April tag detection and drive to.
//Place pixel on correct spot
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

}

