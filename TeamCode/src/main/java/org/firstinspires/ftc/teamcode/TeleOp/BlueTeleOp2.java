package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Autonomous.AprilTagFinder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name="BlueTeleOp2", group="TeleOp")
@Disabled
public class BlueTeleOp2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    final double DESIRED_DISTANCE = 13.5; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for yougfmr robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft;
    private DcMotor RearLeft;
    private DcMotor FrontRight;
    private DcMotor RearRight;
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;

    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection detectedTag = null;     // Used to hold the data for a detected AprilTag
    private int DESIRED_TAG_ID;
    private double drive = 0;
    private double strafe = 0;
    private double turn = 0;
    private Servo pmmA;
    private DcMotor arm;
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;   //  /120;
    int armPosition = 819;
    private Servo pmmF;
    boolean APRIL = true;
    AprilTagFinder tagSearcher = new AprilTagFinder(aprilTag, telemetry);
    boolean Run = true;
    int currentDegree = 0;
    int pmmFCounter = 0;
    int pmmACounter = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");
        pmmF = hardwareMap.get(Servo.class, "pmmfloor");
        pmmA = hardwareMap.get(Servo.class, "pmmA");
        arm = hardwareMap.get(DcMotor.class, "arm");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);

        initAprilTag();// APRIL TAG:
        AprilTagFinder tagSearcher = new AprilTagFinder(aprilTag, telemetry);
        initHardware();


        if (USE_WEBCAM) // APRIL TAG:
            setManualExposure(6, 250);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (gamepad1.right_bumper) {
                if (!slowMode) {
                    slowMode = true;
                    powerMultiplier = 0.2;
                }
            } else {
                if (slowMode) {
                    slowMode = false;
                    powerMultiplier = 1.0;
                }
            }
            FrontLeft.setPower(leftFrontPower * powerMultiplier);
            FrontRight.setPower(rightFrontPower * powerMultiplier);
            RearLeft.setPower(leftBackPower * powerMultiplier);
            RearRight.setPower(rightBackPower * powerMultiplier);

            //Might have to add (subtract?) seven degrees to each, -7 for zero, to account for the 7 movement in the beginning.
            //See if program works like this first
            //Arm rotation
            /*
            double a = -gamepad1.left_stick_y;
            double l = gamepad1.left_stick_x;
            double y = gamepad1.right_stick_x;

            basicMovement(a, l, y);

             */
            if (gamepad2.right_bumper) {
                armMovement(7);    //
            }
            if (gamepad2.dpad_right) {
                armMovement(207);    //
            }
            if (gamepad2.dpad_down) {
                armMovement(178);   //
            }
            if (gamepad2.dpad_left) {
                armMovement(150);  //
            }
            if (gamepad2.dpad_up) {
                armMovement(140);  //
            }
            if (gamepad2.left_bumper) {
                armMovement(0);   //
                pmmF(0.0); // makes it so moving the arm to zero automatically retracts pmmF. Remove if perferable.
            }
            if (gamepad2.left_stick_button) {
                armMovement(90);   //
            }

            //Manipulates pmmF
            if (gamepad2.x) {
                if (currentDegree >= 7)
                    pmmF(0.62);
            }   else if (currentDegree <= 6) {
                pmmF(0.0);
            }
            if (gamepad2.y) {
                pmmF(0.0);
            }
            //ensures pmmF cannot be closed if pmmA is on the ground. See if else necessary under any circumstances.
            //  int pmmACounter = 0;

            //Manipulates pmmA
        }
        if (gamepad2.a) {
            pmmA(0.55);
        }
        if (gamepad2.b) {
            pmmA(0.0);
        }

        //elses prevent changing Apriltag lock on until previous is finished or cancelled
        //Makes robot drive toward apriltag
        if (gamepad1.x) {
            approachTag(1);
        } else if (gamepad1.y) {
            approachTag(2);
        } else if (gamepad1.b) {
            approachTag(3);
        }
//new April tag code to test.
        if (gamepad1.dpad_left) {
            approachTag2(1);
        } else if (gamepad1.dpad_up) {
            approachTag2(2);
        } else if (gamepad1.dpad_right) {
            approachTag2(3);
        }

// only difference in red tele is the detected april tags.

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //   telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        //    telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Servo Position", pmmF.getPosition());
        telemetry.update();
    }

    public void moveAprilRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPow = x - y - yaw;
        double rightFrontPow = x + y + yaw;
        double leftBackPow = x + y - yaw;
        double rightBackPow = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPow), Math.abs(rightFrontPow));
        max = Math.max(max, Math.abs(leftBackPow));
        max = Math.max(max, Math.abs(rightBackPow));

        if (max > 1.0) {
            leftFrontPow /= max;
            rightFrontPow /= max;
            leftBackPow /= max;
            rightBackPow /= max;
        }

        // Send powers to the wheels.
        FrontLeft.setPower(leftFrontPow);
        FrontRight.setPower(rightFrontPow);
        RearLeft.setPower(leftBackPow);
        RearRight.setPower(rightBackPow);
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

    private void initHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);
    }

    private void setManualExposure(int exposureMS, int gain) {
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
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }

    }

    // not really sure what is wrong with april. And it's late.
    private void approachTag(int DESIRED_TAG_ID) {
        while (APRIL = true) {
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
                break;
            } else if (gamepad1.right_stick_button) {       //not sure if this one works. But I think we want a system we can leave anytime.
                break;
            }
            break;
        }
    }
    boolean targetFound     = false;
    private AprilTagDetection desiredTag = null;
    private void approachTag2(int DESIRED_TAG_ID) {
        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    AprilTagDetection detectedTag = tagSearcher.findTag(DESIRED_TAG_ID);
                    // If the AprilTag is detected the robot drives to it
                    if (detectedTag != null) {
                        // Yes, we want to use this tag.
                        desiredTag = detection;
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
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
            moveAprilRobot(drive, strafe, turn);
            if (detectedTag != null && (detectedTag.ftcPose.range - DESIRED_DISTANCE) <= 0.05) {
                break;
            } else if (gamepad1.right_stick_button) {       //not sure if this one works. But I think we want a system we can leave anytime.
                break;
            }
            break;
        }
    }


    private void armMovement(int degree) {

        while (Run = true) {
            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            //maybe try a sleep? Does it run during sleep? Because that would solve all our problems. Except Apriltags being weird.
            if (gamepad2.right_stick_button) {
                degree = currentDegree;
                break;
            }
            break;
        }

    }

    private void pmmF(double turnValF) {
        pmmF.setDirection(Servo.Direction.REVERSE);
        pmmF.setPosition(turnValF);
    }

    private void pmmA(double turnValA) {
        pmmA.setDirection(Servo.Direction.FORWARD);
        pmmA.setPosition(turnValA);
    }

/*
    private void pmmF(double turnValF) {
        if (pmmFCounter == 0) {
            if(currentDegree >= 7) {
                pmmF.setDirection(Servo.Direction.REVERSE);
                pmmF.setPosition(turnValF);
                pmmFCounter = pmmFCounter + 1;
            }
            else{
                pmmF.setDirection(Servo.Direction.REVERSE);
                pmmF.setPosition(0.0);
                pmmFCounter = pmmFCounter - 1;
            }
        } else if (pmmFCounter == 1) {
            pmmF.setDirection(Servo.Direction.REVERSE);
            pmmF.setPosition(0.0);
            pmmFCounter = pmmFCounter - 1;
        }
    }

    private void pmmA(double turnValA) {
        if (pmmACounter == 0) {
            pmmA.setDirection(Servo.Direction.REVERSE);
            pmmA.setPosition(turnValA);
            pmmACounter = pmmACounter + 1;
        } else if (pmmACounter == 1) {
            pmmA.setDirection(Servo.Direction.REVERSE);
            pmmA.setPosition(0.0);
            pmmACounter = pmmACounter - 1;
        }
    }
*/
}