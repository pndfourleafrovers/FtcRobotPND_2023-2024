package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class pmmA_Tele extends LinearOpMode {

    private Servo pmmA;
    private DcMotor arm;
    static final int TICKS_PER_MOTOR_REV = 1425;
    static final double TICKS_PER_GEAR_REV = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV / 120;
    int armPosition = 819;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        pmmA = hardwareMap.get(Servo.class, "pmmA");

        arm = hardwareMap.get(DcMotor.class, "arm");
        waitForStart();



        while (opModeIsActive()) {

            arm.setDirection(DcMotor.Direction.FORWARD);

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(0.5);
            arm.setTargetPosition(TICKS_PER_DEGREE * 7);

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
        }
    }
}