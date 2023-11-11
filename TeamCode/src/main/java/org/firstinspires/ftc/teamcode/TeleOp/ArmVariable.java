package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mixed.AprilTagFinder;
import org.firstinspires.ftc.teamcode.Mixed.AprilTagFinder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="TeleOpFinal", group="TeleOp")
//@Disabled
public class ArmVariable extends LinearOpMode {

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
    int currentDegree = 0;
    static final int     TICKS_PER_MOTOR_REV    = 1425;
    static final int     TICKS_PER_GEAR_REV    = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV/360;   //  /120;
    int armPosition = 819;
    private Servo pmmF;
    boolean APRIL = true;
    boolean Run = true;
    private IMU imu = null;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        pmmA = hardwareMap.get(Servo.class, "pmmA");
        arm = hardwareMap.get(DcMotor.class, "arm");

        initHardware();



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;


            double armMove =  -gamepad2.left_stick_y;
            telemetry.addData("Arm Position", arm.getCurrentPosition()/TICKS_PER_DEGREE);

            //      if (max2 > 0.3) {
            //           armPower /= max;
            //    }

            double armPower=armMove;
/**
 * Test code
 */
/*
            double Sine = Math.sin(armPower);
            double doubleSin = Math.pow(Sine, 0.5);
            int intSin = (int)doubleSin;
            arm.setTargetPosition(intSin*TICKS_PER_DEGREE);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower * 0.2);
            currentDegree = arm.getCurrentPosition() / TICKS_PER_DEGREE;
            if(arm.getCurrentPosition()==intSin*TICKS_PER_DEGREE){
                int holdPosition = arm.getCurrentPosition()/TICKS_PER_DEGREE;
                armMovement(holdPosition);
            }

 */




            arm.setPower(armPower * 0.2);
            currentDegree = arm.getCurrentPosition() / TICKS_PER_DEGREE;
            if(arm.getCurrentPosition()==currentDegree*TICKS_PER_DEGREE){
                int holdPosition = arm.getCurrentPosition()/TICKS_PER_DEGREE;
                armMovement(holdPosition, 0.2);
            }




            if (gamepad2.right_bumper) {
                armMovement(7, 0.7);    //0
            }
            if (gamepad2.dpad_right) {
                armMovement(207, 0.7);    //200   207
            }
            if (gamepad2.dpad_down) {
                armMovement(193, 0.7);   //171     178
            }
            if (gamepad2.dpad_left) {
                armMovement(178, 0.7);  //143      150
            }
            if (gamepad2.dpad_up) {
                armMovement(140, 0.7);  //113       120
            }
            if (gamepad2.left_bumper) {
                armMovement(0, 0.7);   //-7          0
            }
            if (gamepad2.left_stick_button) {
                armMovement(90, 0.7);   //-7          0
            }
            /**
             * Test Below
             * Makes it so robot takes "snapshot" of current Position and saves it
             */
            /*
            if (gamepad2.right_stick_button) {
                int holdPosition = arm.getCurrentPosition()/TICKS_PER_DEGREE;
                armMovement(holdPosition, 0.2);
            }

             */
/**
 * Test Below
 * Togglable version, not preferable
 */
/*
            if (gamepad2.right_stick_button) {
                double armMove =  -gamepad2.left_stick_y;
                telemetry.addData("Arm Position", arm.getCurrentPosition()/TICKS_PER_DEGREE);
                double armPower=armMove;
                arm.setPower(armPower * 0.2);
                currentDegree = arm.getCurrentPosition() / TICKS_PER_DEGREE;
                if(arm.getCurrentPosition()==currentDegree*TICKS_PER_DEGREE){
                    int holdPosition = arm.getCurrentPosition()/TICKS_PER_DEGREE;
                    armMovement(holdPosition, 0.2);
                }
            }
*/
/**
 * Test Below
 * set version, not preferable
 * Would need a separate one for going back down.
 */
/*
            if (gamepad2.right_stick_button) {
                armMovement(currentDegree++, 0.2);
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



    private int armMovement(int degree, double power) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
            /**
             * Test Below
             */
            //    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /**
             * Test Below
             */
            //   degree = currentDegree;
            //maybe try a sleep? Does it run during sleep? Because that would solve all our problems. Except Apriltags being weird.
            //   if (gamepad2.right_stick_button) {
            //      degree = currentDegree;
            //      break;
            //   }
            break;
        }
        return currentDegree;
    }
   /*
    private int centerPassage(int degree, int distance) {
        armMovement(degree);
        sleep(1000);
        driveForward(12, 1);

    }
*/



}
/*
double armMove =  -gamepad2.left_stick_y;

while (opModeIsActive() && (arm.isBusy())) {
        telemetry.addData("FrontLeft Position", arm.getCurrentPosition()/TICKS_PER_DEGREE);
        }

 */


