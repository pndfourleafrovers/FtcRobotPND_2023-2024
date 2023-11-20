package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Random.Mixed.AprilTagFinder;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name="TeleOpFinal", group="TeleOp")
//@Disabled
public class TeleOpFinal extends LinearOpMode {

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
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;   //  /120;
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
        FrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        RearRight = hardwareMap.get(DcMotor.class, "Right_rear");
        pmmF = hardwareMap.get(Servo.class, "pmmfloor");
        pmmA = hardwareMap.get(Servo.class, "pmmA");
        arm = hardwareMap.get(DcMotor.class, "arm");
      //  drone = hardwareMap.get(DcMotor.class, "drone");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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
            telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
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
            //    max = Math.max(max, Math.abs(armPower));

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
            //        if (max > 1.0) {
            //            leftFrontPower /= max;
            //            rightFrontPower /= max;
            //           leftBackPower /= max;
            //           rightBackPower /= max;
            //      }

            FrontLeft.setPower(leftFrontPower * powerMultiplier);
            FrontRight.setPower(rightFrontPower * powerMultiplier);
            RearLeft.setPower(leftBackPower * powerMultiplier);
            RearRight.setPower(rightBackPower * powerMultiplier);

            if (gamepad2.right_bumper) {
                armMovementBack(7, 0.7);    //0
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
                armMovementBack(0, 0.7);   //-7          0
            }
            if (gamepad2.left_stick_button) {
                armMovement(90);   //-7          0
            }

            if (gamepad2.right_stick_button) {
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                double armMove = -gamepad2.left_stick_y;
                telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
                telemetry.update();
                double armPowerV = armMove;
                if (armPowerV < -0.1) {
                    armPowerV = -0.1;
                } else if (armPowerV > 0.2) {
                    armPowerV = 0.2;
                } else if (armPowerV == 0) {
                    armPowerV = 0.0000001;
                }
                arm.setPower(armPowerV);
            }
            if (gamepad2.back) {
                int holdPosition = arm.getCurrentPosition() / TICKS_PER_DEGREE;
                telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
                telemetry.update();
                armMovementBack(holdPosition, 0.2);
            }
            /**
             * Test Below
             */
            //      if (gamepad2.right_stick_button) {
            //          int holdPosition = arm.getCurrentPosition()/TICKS_PER_DEGREE;
            //          armMovement(holdPosition);
            //      }

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

            if (gamepad1.dpad_up) {
                driveForward(28, 1);
            }
            if (gamepad1.dpad_left) {
                strafeLeft(3, 0.5);
            }
            if (gamepad1.dpad_right) {
                strafeRight(3, 0.5);
            }

            if (gamepad1.x) {
                turnToHeading(90);
            }
            if (gamepad1.y) {
                turnToHeading(180);
            }

            if (gamepad1.a) {
                armMovementBack(57, 0.6);
                sleep(800);
                driveBackward(24, 1);
                armMovementBack(7, 0.7);
            }
            if (gamepad1.b) {
                armMovementBack(2, 0.5);
                pmmF(0.62);
                sleep(500);
                pmmA(0.55);
                sleep(200);
                armMovementBack(7, 0.7);
            }
            //drone lauch
            /*
            if (gamepad1.left_bumper) {
                droneLaunch(0);
            } else{
                droneLaunch(1);
            }

             */
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //   telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        //    telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Servo Position", pmmF.getPosition());
        telemetry.addData("Arm Position", arm.getCurrentPosition() / TICKS_PER_DEGREE);
        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
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


    private int armMovementBack(int degree, double power) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
            degree = currentDegree;
            telemetry.addData("Arm Position:", currentDegree);
            telemetry.update();
            break;
        }
        return currentDegree;
    }
    private int armMovement(int degree) {
        //    Run = true;
        while (Run = true) {

            arm.setTargetPosition(TICKS_PER_DEGREE * (degree - currentDegree));
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmPowerCalc(degree);
            degree = currentDegree;
            telemetry.addData("Arm Position:", currentDegree);
            telemetry.update();
            break;
        }
        return currentDegree;
    }

    public double getDegree() {
        double armPos;
        armPos = arm.getCurrentPosition()/TICKS_PER_DEGREE;
        return armPos;
    }


    public void ArmPowerCalc(int CalcDegree) {
        double ArmMult = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getDegree() - CalcDegree) > 0.5) { // Allow for a small error margin

            double error = CalcDegree - getDegree(); // Calculate the error

            // Calculate wheel powers based on error
            double armPower = error * ArmMult;
            double Low = 0.2;
            if(armPower<Low)
                armPower = Low;
            arm.setPower(armPower);
        }
    }
   /*
    private int centerPassage(int degree, int distance) {
        armMovement(degree);
        sleep(1000);
        driveForward(12, 1);

    }
*/

    private void pmmF(double turnValF) {
        pmmF.setDirection(Servo.Direction.REVERSE);
        pmmF.setPosition(turnValF);
        //  return turnValF;
    }

    private void pmmA(double turnValA) {
        pmmA.setDirection(Servo.Direction.FORWARD);
        pmmA.setPosition(turnValA);
        //  return turnValA;
    }
/*
    private void droneLaunch(double turnValD) {
        .setDirection(Servo.Direction.FORWARD);
        .setPosition(turnValD);
        //  return turnValA;
    }
    */
    public void driveBackward(double distance, double power) {
        driveForward(-distance, power);
    }
    public void driveForward(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);
        // Set the target positions for each motor
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + ticksToMove);
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + ticksToMove);
        RearLeft.setTargetPosition(RearLeft.getCurrentPosition() + ticksToMove);
        RearRight.setTargetPosition(RearRight.getCurrentPosition() + ticksToMove);

        // Set the mode to RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        RearLeft.setPower(power);
        RearRight.setPower(power);
        while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || RearLeft.isBusy() || RearRight.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("FrontLeft Position", FrontLeft.getCurrentPosition());
            telemetry.addData("FrontRight Position", FrontRight.getCurrentPosition());
            telemetry.addData("RearLeft Position", RearLeft.getCurrentPosition());
            telemetry.addData("RearRight Position", RearRight.getCurrentPosition());
            telemetry.update();
        }
        // Stop all the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRight(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
        ticksToMove = (int) (distance * ticksPerInch);

        // Positive distance means strafe right, negative means strafe left
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        RearLeft.setTargetPosition(RearLeft.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        RearRight.setTargetPosition(RearRight.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));

        // Set the mode to RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        RearLeft.setPower(power);
        RearRight.setPower(power);

        // Wait for the motors to finish
        while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || RearLeft.isBusy() || RearRight.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("FrontLeft Position", FrontLeft.getCurrentPosition());
            telemetry.addData("FrontRight Position", FrontRight.getCurrentPosition());
            telemetry.addData("RearLeft Position", RearLeft.getCurrentPosition());
            telemetry.addData("RearRight Position", RearRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft(double distance, double power) {
        int ticksToMove;
        double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI));
        ticksToMove = (int) (distance * ticksPerInch);

        // Positive distance means strafe left, negative means strafe right
        FrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));
        FrontRight.setTargetPosition(FrontRight.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        RearLeft.setTargetPosition(RearLeft.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? 1 : -1)));
        RearRight.setTargetPosition(RearRight.getCurrentPosition() + (int) (ticksToMove * (distance > 0 ? -1 : 1)));

        // Set the mode to RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        RearLeft.setPower(power);
        RearRight.setPower(power);

        // Wait for the motors to finish
        while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || RearLeft.isBusy() || RearRight.isBusy())) {
            // Optionally provide telemetry updates
            telemetry.addData("FrontLeft Position", FrontLeft.getCurrentPosition());
            telemetry.addData("FrontRight Position", FrontRight.getCurrentPosition());
            telemetry.addData("RearLeft Position", RearLeft.getCurrentPosition());
            telemetry.addData("RearRight Position", RearRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);

        // Reset the motor modes back to RUN_USING_ENCODER
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void turnToHeading(int targetHeading) {
      //  int ticksToMove;
      //  double ticksPerInch = (537.7 / ((96 / 25.4) * Math.PI)); //537.7 encoder ticks, wheel is 96mm converting to inches divide by 25.4, per one circumference of the wheel
      //  ticksToMove = (int) (distance * ticksPerInch);
        double turnKp = 0.01; // This is a proportionality constant. You may need to tune this value.
        while (Math.abs(getHeading() - targetHeading) > 1) { // Allow for a small error margin

            double error = targetHeading - getHeading(); // Calculate the error

            // Calculate wheel powers based on error
            double turnPower = error * turnKp;

            double leftFrontPower = -turnPower;
            double rightFrontPower = +turnPower;
            double leftBackPower = -turnPower;
            double rightBackPower = +turnPower;

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

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Send powers to the wheels.
            FrontLeft.setPower(leftFrontPower);
            FrontRight.setPower(rightFrontPower);
            RearLeft.setPower(leftBackPower);
            RearRight.setPower(rightBackPower);

            while (opModeIsActive() && (FrontLeft.isBusy() || FrontRight.isBusy() || RearLeft.isBusy() || RearRight.isBusy())) {
                // Optionally provide telemetry updates
                telemetry.addData("FrontLeft Position", FrontLeft.getCurrentPosition());
                telemetry.addData("FrontRight Position", FrontRight.getCurrentPosition());
                telemetry.addData("RearLeft Position", RearLeft.getCurrentPosition());
                telemetry.addData("RearRight Position", RearRight.getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();

        }
        // Stop all the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
/*
double armMove =  -gamepad2.left_stick_y;

while (opModeIsActive() && (arm.isBusy())) {
        telemetry.addData("FrontLeft Position", arm.getCurrentPosition()/TICKS_PER_DEGREE);
        }

 */

