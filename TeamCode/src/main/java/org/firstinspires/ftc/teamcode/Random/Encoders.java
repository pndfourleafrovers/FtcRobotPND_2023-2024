package org.firstinspires.ftc.teamcode.Random;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="Encoders", group="Autonomous")
@Disabled
public class Encoders extends LinearOpMode {
    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx RearLeft;
    private DcMotorEx RearRight;
    //Ex allows us to setVelocity(); See if wanted/preferred. Velocity, when values opposite, enables turning.
    static final double     TICKS_PER_MOTOR_REV    = 520;
    //537.7
    static final double     DRIVE_GEAR_REDUCTION    = 1.5;
    static final double     WHEEL_CIRCUMFERENCE_IN  = 2.5 * Math.PI;
    //96mm
    static final double     TICKS_PER_WHEEL_REV    = TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     TICKS_PER_IN           = TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_IN;
    // the slashes here indicate my unsureness over Drive Gear Reduction, mostly due to not knowing what on earth our robot is gonna look like.
    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotorEx.class, "Left_front");
        FrontRight = hardwareMap.get(DcMotorEx.class, "Right_front");
        RearLeft = hardwareMap.get(DcMotorEx.class, "Left_rear");
        RearRight = hardwareMap.get(DcMotorEx.class, "Right_rear");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        RearRight.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Resets tick count of encoders to 0

        int FrontLeftTarget = (int)(12 * TICKS_PER_IN);  //12 is in place of choosen distance in inches.
        int FrontRightTarget = (int)(12 * TICKS_PER_IN);
        int RearLeftTarget = (int)(12 * TICKS_PER_IN);
        int RearRightTarget = (int)(12 * TICKS_PER_IN);
        double FrLTPS = (80/ 60) * TICKS_PER_WHEEL_REV;        // TPS means Ticks Per Second. RPM/60 to convert to seconds
        double FrRTPS = (80/ 60) * TICKS_PER_WHEEL_REV;        // 80 indictes RMP, can be 0-160RPM
        double ReLTPS = (80/ 60) * TICKS_PER_WHEEL_REV;
        double ReRTPS = (80/ 60) * TICKS_PER_WHEEL_REV;
        waitForStart();

        FrontLeft.setTargetPosition(FrontLeftTarget);
        FrontRight.setTargetPosition(FrontRightTarget);
        RearLeft.setTargetPosition(RearLeftTarget);
        RearRight.setTargetPosition(RearRightTarget);

        FrontLeft.setVelocity(FrLTPS);
        FrontRight.setVelocity(FrRTPS);
        RearLeft.setVelocity(ReLTPS);
        RearRight.setVelocity(ReRTPS);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy() && RearLeft.isBusy() && RearRight.isBusy())){
            //Keeps Robot from continuing program before all motors have reached target. Not necessarily necessary.
            telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
            telemetry.addData("Frontright", FrontRight.getCurrentPosition());
            telemetry.addData("Rearleft", RearLeft.getCurrentPosition());
            telemetry.addData("Rearright", RearRight.getCurrentPosition());;
            telemetry.update();
        }
    }
}


/*

//Final note. Depending on design, you might need to add in the reverse stuff for drivetrain navigation
Goes after reset thing.
        int FrontLeftTarget = (int)(distance in mm * COUNTS_PER_MM);
        int FrontRightTarget = (int)(distance in mm * COUNTS_PER_MM);
        int RearLeftTarget = (int)(distance in mm * COUNTS_PER_MM);
        int RearRightTarget = (int)(distance in mm * COUNTS_PER_MM);
        double FrLTPS = (Chosen RPM/ 60) * COUNTS_PER_WHEEL_REV;         Means Front Left Ticks Per Second. /60 to convert to seconds
        double FrRTPS = (Chosen RPM/ 60) * COUNTS_PER_WHEEL_REV;
        double ReLTPS = (Chosen RPM/ 60) * COUNTS_PER_WHEEL_REV;
        double ReRTPS = (Chosen RPM/ 60) * COUNTS_PER_WHEEL_REV;
        waitForStart();


        leftmotor.setTargetPosition(leftTarget);
        rightmotor.setTargetPosition(rightTarget);

        Run to postion code. Already up there though.

        FrontLeft.setVelocity(FrLTPS);
        FrontRight.setVelocity(FrRTPS);
        RearLeft.setVelocity(ReLTPS);
        RearRight.setVelocity(ReRTPS);
        setVelocity can be put on opposing values to turn the robot. Requires DriveMotorEx thing.
 */
/*
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        FrontLeft.setTargetPosition(1000);
        FrontRight.setTargetPosition(1000);
        RearLeft.setTargetPosition(1000);
        RearRight.setTargetPosition(1000);
        // 1000 indicates ticks. Can be altered. Sets Target

        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Tells robot to move to target

        FrontLeft.setPower(0.8);
        FrontRight.setPower(0.8);
        RearLeft.setPower(0.8);
        RearRight.setPower(0.8);
        // Sets speed/power of the robot while moving to the target. Potentially slow down to reduce slide after movement ended
*/