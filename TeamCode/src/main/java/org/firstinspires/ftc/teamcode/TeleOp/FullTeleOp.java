package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="FullTeleOp", group="TeleOp")
@Disabled
public class FullTeleOp extends LinearOpMode{

        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor FrontLeft;
        private DcMotor RearLeft;
        private DcMotor FrontRight;
        private DcMotor RearRight;
        private Servo pmmA;
        private DcMotor arm;
        static final int TICKS_PER_MOTOR_REV = 1425;
        static final double TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
        static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120;
        int armPosition = 819;
        private Servo Grabber;
        @Override
        public void runOpMode() {

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            FrontLeft  = hardwareMap.get(DcMotor.class, "Left_front");
            RearLeft  = hardwareMap.get(DcMotor.class, "Left_rear");
            FrontRight = hardwareMap.get(DcMotor.class, "Right_front");
            RearRight = hardwareMap.get(DcMotor.class, "Right_rear");

            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            RearLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            RearRight.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                arm.setDirection(DcMotor.Direction.FORWARD);

                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(0.5);
                arm.setTargetPosition(TICKS_PER_DEGREE * 7);
                Grabber.setPosition(1);
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral =  gamepad1.left_stick_x;
                double yaw     =  gamepad1.right_stick_x;

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower  /= max;
                    rightFrontPower /= max;
                    leftBackPower   /= max;
                    rightBackPower  /= max;
                }

                // Send calculated power to wheels
                FrontLeft.setPower(leftFrontPower);
                FrontRight.setPower(rightFrontPower);
                RearLeft.setPower(leftBackPower);
                RearRight.setPower(rightBackPower);
                if(gamepad2.x) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 7);
                    pmmA.setPosition(1);
                }
                else if(gamepad2.y){
                    arm.setTargetPosition(TICKS_PER_DEGREE * 207);
                    pmmA.setPosition(0);
                }
                //the end is a test to see if that works, please change to something else if possible. Which it is.
                else if(gamepad2.y && gamepad2.x){
                    arm.setTargetPosition(TICKS_PER_DEGREE * 178);
                    pmmA.setPosition(0);
                }
                else if(gamepad2.dpad_down){
                    arm.setTargetPosition(TICKS_PER_DEGREE * 150);
                    pmmA.setPosition(0);
                }
                else if(gamepad2.dpad_up){
                    arm.setTargetPosition(TICKS_PER_DEGREE * 120);
                }
                else if(gamepad2.a) {
                    Grabber.setPosition(1);
                }
                    // Servos operate 0-180 degrees by a 0-1 metric. This sets servo position to 180 degrees.
                else if(gamepad2.b) {
                    Grabber.setPosition(0.0);
                }

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Servo Position", Grabber.getPosition());
                telemetry.update();
            }
        }
    }
