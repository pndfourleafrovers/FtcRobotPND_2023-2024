package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Autonomous.AllQuads2;

public class Arm extends AllQuads2 {
    public DcMotor arm;
    private Servo pmmA;
    private Servo pmmF;
    ElapsedTime timer = new ElapsedTime();
    public void initArm() {
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void initServos() {
        pmmF = hardwareMap.get(Servo.class, "pmmfloor");
        pmmA = hardwareMap.get(Servo.class, "pmmA");
    }
    public void resetArm() {
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        sleep(2000);
        arm.setTargetPosition(TICKS_PER_DEGREE * 7);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
    public void dropArmToZeroAndReset() {
        arm.setTargetPosition(TICKS_PER_DEGREE * 0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveArmToBackDrop() {
        arm.setTargetPosition(TICKS_PER_DEGREE * 210);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
        //give the arm time to get to the backdrop
        sleep(2000);
    }
    public void HoldArmOffGround() {
        // Lift the ARM 7 degrees off of the ground and hold it there
        arm.setTargetPosition(TICKS_PER_DEGREE * 7);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.5);
    }
}
