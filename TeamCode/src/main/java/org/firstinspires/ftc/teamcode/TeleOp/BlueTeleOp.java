package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Random.Mixed.AprilTagFinder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="BlueTeleOp", group="TeleOp")
@Disabled
public class BlueTeleOp extends LinearOpMode {

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
    private boolean slowMode = false;
    private double powerMultiplier = 1.0;
    private DcMotor FrontRight;
    private DcMotor RearRight;
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
    int currentDegree;
    static final int     TICKS_PER_MOTOR_REV    = 1425;
    static final int     TICKS_PER_GEAR_REV    = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV/360;   //  /120;
    int armPosition = 819;
    private Servo pmmF;
    boolean APRIL = true;
    AprilTagFinder tagSearcher = new AprilTagFinder(aprilTag, telemetry);
    boolean Run = true;


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

        AprilTagFinder tagSearcher = new AprilTagFinder(aprilTag, telemetry);
        initHardware();


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


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;

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
                    armMovement(7);    //0
                }
                if (gamepad2.dpad_right) {
                    armMovement(207);    //200   207
                }
                if (gamepad2.dpad_down) {
                    armMovement(193);   //171     178
                }
                if (gamepad2.dpad_left) {
                    armMovement(178);  //143      150
                }
                if (gamepad2.dpad_up) {
                    armMovement(140);  //113       120
                }
                if (gamepad2.left_bumper) {
                    armMovement(0);   //-7          0
                }
                if (gamepad2.left_stick_button) {
                    armMovement(90);   //-7          0
                }

                //Manipulates pmmF
                if (gamepad2.y) {
                    pmmF(0.0);
                }
                if (gamepad2.x) {
                    pmmF(0.62);
                }


                //Manipulates pmmA
                if (gamepad2.b) {
                    pmmA(0.0);
                }
                if (gamepad2.a) {
                    pmmA(0.55);
                }


                //Makes robot drive toward apriltag

            /*
            if (gamepad1.x) {
                approachTag(1);
            } else if (gamepad1.y) {
                approachTag(2);
            } else if (gamepad1.b) {
                approachTag(3);
            }



        }
*/
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //   telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //    telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Servo Position", pmmF.getPosition());
            telemetry.update();
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



        private int armMovement(int degree) {
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
            return currentDegree;
        }


        private void armMovement2(int degrees) {
            arm.setTargetPosition(TICKS_PER_DEGREE * degrees);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            sleep(2000);
            pmmA.setDirection(Servo.Direction.FORWARD);
            pmmA.setPosition(0.0);
            sleep(10);
            arm.setTargetPosition(TICKS_PER_DEGREE * -degrees);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            sleep(2000);
        }

        private void pmmF(double turnValF) {
            pmmF.setDirection(Servo.Direction.REVERSE);
            pmmF.setPosition(turnValF);
            //  return turnValF;
        }

        private void pmmA(double turnValA) {
            pmmA.setDirection(Servo.Direction.FORWARD);
            pmmA.setPosition(turnValA);
            //    return turnValA;
        }

    }