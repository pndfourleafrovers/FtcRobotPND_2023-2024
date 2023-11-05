package org.firstinspires.ftc.teamcode.Archive.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous (name="pmmA_Auto", group="Autonomous")
public class pmmA_Auto extends LinearOpMode {

    private Servo pmmA;
    private DcMotor arm;
    static final int     TICKS_PER_MOTOR_REV    = 1425;
    static final double     TICKS_PER_GEAR_REV    = TICKS_PER_MOTOR_REV * 3;
    static final int TICKS_PER_DEGREE = TICKS_PER_MOTOR_REV/120;
    int armPosition = 819;
    private ElapsedTime runtime = new ElapsedTime();
//3:1 gear ratio
    @Override
    public void runOpMode() {

        pmmA = hardwareMap.get(Servo.class, "pmmA");

        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Set at beginning
        arm.setPower(0.5);
        arm.setTargetPosition(TICKS_PER_DEGREE*7);
        waitForStart();



        while (opModeIsActive()){
            //Set when placing pixel
            arm.setTargetPosition(TICKS_PER_DEGREE*207);
            //Release pixel
            pmmA.setPosition(0.0);



        }
        telemetry.addData("Servo Position", pmmA.getPosition());
        telemetry.update();
    }
}
