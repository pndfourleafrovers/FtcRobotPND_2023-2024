package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled
@TeleOp(name="pmmA_Tele", group="TeleOp")
public class pmmA_Tele extends LinearOpMode {

    private Servo pmmA;
    private DcMotor arm;
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final int TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_GEAR_REV / 360;
    int armPosition = 819;
    private ElapsedTime runtime = new ElapsedTime();
    boolean Run = true;

    @Override
    public void runOpMode() {

        pmmA = hardwareMap.get(Servo.class, "pmmA");

        arm = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();


        while (opModeIsActive()) {
            //pmmA.setDirection(Servo.Direction.);
            arm.setDirection(DcMotor.Direction.FORWARD);

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.setPower(0.0);
            if (gamepad1.x) {
                arm.setTargetPosition(TICKS_PER_DEGREE * 7);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);
            }
            /*
            if (gamepad1.x) {
                while (Run = true) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 7);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                   // pmmA.setPosition(0.62);
                    if (gamepad1.right_stick_button) {
                        //alternative: reverse directions and move a positive value. Remember to reset positive direction afterwards,
                        //assuming it doesn't do it automatically.
                        arm.setTargetPosition(TICKS_PER_DEGREE * -7);
                        //pmmA.setPosition(0.0);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        break;
                    }
                }
            }
            */

            else if (gamepad1.y) {
                while (Run = true) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 207);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                    if (gamepad1.right_stick_button) {
                        arm.setTargetPosition(TICKS_PER_DEGREE * -207);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        break;
                    }
                }
            }
            else if (gamepad1.a) {
                while (Run = true) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 178);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                    if (gamepad1.right_stick_button) {
                        arm.setTargetPosition(TICKS_PER_DEGREE * -178);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        break;
                    }
                }
            }
            else if (gamepad1.b) {
                while (Run = true) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 150);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                    if (gamepad1.right_stick_button) {
                        arm.setTargetPosition(TICKS_PER_DEGREE * -150);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        break;
                    }
                }
            }
            else if (gamepad1.dpad_right) {
                while (Run = true) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 120);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                    if (gamepad1.right_stick_button) {
                        arm.setTargetPosition(TICKS_PER_DEGREE * -120);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        break;
                    }
                }
            }
            else if (gamepad1.dpad_down) {
                while (Run = true) {
                    arm.setTargetPosition(TICKS_PER_DEGREE * 0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.5);
                    if (gamepad1.right_stick_button) {
                        arm.setTargetPosition(TICKS_PER_DEGREE * -0);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        break;
                    }
                }
            }
            if(gamepad2.x){
                armMovement(7);
            }
            else if (gamepad2.y) {
                armMovement(207);
            }
            else if (gamepad2.a) {
                armMovement(178);
            }
            else if (gamepad2.b) {
                armMovement(150);
            }
            else if (gamepad2.dpad_right) {
                armMovement(120);
            }
            else if (gamepad2.dpad_down) {
                armMovement(0);
            }
        }
    }

    private void armMovement (int degree){
        while (Run = true) {
            arm.setTargetPosition(TICKS_PER_DEGREE * degree);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            // pmmA.setPosition(0.62);
            if (gamepad2.right_stick_button) {
                //alternative: reverse directions and move a positive value. Remember to reset positive direction afterwards,
                //assuming it doesn't do it automatically.
                arm.setTargetPosition(TICKS_PER_DEGREE * -degree);
                //pmmA.setPosition(0.0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            }
        }

    }
}
