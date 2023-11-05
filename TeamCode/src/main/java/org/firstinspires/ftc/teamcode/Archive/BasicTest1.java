package org.firstinspires.ftc.teamcode.Archive;
//Will act as proof of movement and turning capabilities in combination

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Objects.Odometry;

@Disabled
@Autonomous(name="BasicT", group="Autonomous")
public class BasicTest1 extends LinearOpMode {
    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx RearLeft;
    private DcMotorEx RearRight;
    private IMU imu;      // Control/Expansion Hub IMU

    private double headingError = 0;

    private double FrontLeftSpeed = 0;
    private double FrontRightSpeed = 0;
    private double RearLeftSpeed = 0;
    private double RearRightSpeed = 0;
    private int FrontLeftTarget = 0;
    private int FrontRightTarget = 0;
    private int RearLeftTarget = 0;
    private int RearRightTarget = 0;

    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.1;
    static final double HEADING_THRESHOLD = 0.5;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;
    static final double TICKS_PER_MOTOR_REV = 520;
    static final double DRIVE_GEAR_REDUCTION = 1.5;
    static final double WHEEL_CIRCUMFERENCE_IN = 2.5 * Math.PI;
    static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double TICKS_PER_IN = TICKS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_IN;
    private int RPM = 100;

    @Override
    public void runOpMode() {
        Odometry OdoMethods = new Odometry();
        FrontLeft = hardwareMap.get(DcMotorEx.class, "Left_front");
        RearLeft = hardwareMap.get(DcMotorEx.class, "Left_rear");
        FrontRight = hardwareMap.get(DcMotorEx.class, "Right_front");
        RearRight = hardwareMap.get(DcMotorEx.class, "Right_rear");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        RearLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        RearLeft.setDirection(DcMotor.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //displays angle while waiting. Good for setup.
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", OdoMethods.getHeading());
            telemetry.update();
        }
//Prior was basic setup. Following is movement stuff. Code will be split between Pure odometry and encoders+odometry
        //Odometry
/*
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        OdoMethods.driveStraight(DRIVE_SPEED, 20, 0.0);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, 360.0);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 360.0, 0.5);

        OdoMethods.driveStraight(DRIVE_SPEED, 14.0, 0.0);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, 90.0);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 90.0, 0.5);

        OdoMethods.driveStraight(DRIVE_SPEED, 44.0, 0.0);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, 90.0);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 90.0, 0.5);

        OdoMethods.driveStraight(DRIVE_SPEED, 18.0, 0.0);    // In inches
        OdoMethods.turnToHeading(TURN_SPEED, -90.0);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, -90.0, 0.5);


*/


        //Encoders + Odometry
        //Figure out wait for start stuff and where to put things
        int FrontLeftTarget = (int) (20 * TICKS_PER_IN);  //12 is in place of choosen distance in inches.
        int FrontRightTarget = (int) (20 * TICKS_PER_IN);
        int RearLeftTarget = (int) (20 * TICKS_PER_IN);
        int RearRightTarget = (int) (20 * TICKS_PER_IN);
        double FrLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // TPS means Ticks Per Second. RPM/60 to convert to seconds
        double FrRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // RPM indictes RPM, can be 0-160RPM
        double ReLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;
        double ReRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;
        waitForStart();

        FrontLeft.setTargetPosition(FrontLeftTarget);
        FrontRight.setTargetPosition(FrontRightTarget);
        RearLeft.setTargetPosition(RearLeftTarget);
        RearRight.setTargetPosition(RearRightTarget);

        FrontLeft.setVelocity(FrLTPS);
        FrontRight.setVelocity(FrRTPS);  //Set Velocity requires DcMotorEx
        RearLeft.setVelocity(ReLTPS);    // Velocity can also be used to turn potentially
        RearRight.setVelocity(ReRTPS);

        OdoMethods.turnToHeading(TURN_SPEED, 360.0);               // In Degrees
        OdoMethods.holdHeading(TURN_SPEED, 360.0, 0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy() && RearLeft.isBusy() && RearRight.isBusy())) {
            //Keeps Robot from continuing program before all motors have reached target. Not necessarily necessary.
            telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
            telemetry.addData("Frontright", FrontRight.getCurrentPosition());
            telemetry.addData("Rearleft", RearLeft.getCurrentPosition());
            telemetry.addData("Rearright", RearRight.getCurrentPosition());
            ;
            telemetry.update();

            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            FrontLeftTarget = (int) (14 * TICKS_PER_IN);  //12 is in place of choosen distance in inches.
            FrontRightTarget = (int) (14 * TICKS_PER_IN);
            RearLeftTarget = (int) (14 * TICKS_PER_IN);
            RearRightTarget = (int) (14 * TICKS_PER_IN);
            FrLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // TPS means Ticks Per Second. RPM/60 to convert to seconds
            FrRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // RPM indictes RMP, can be 0-160RPM
            ReLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;
            ReRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;

            FrontLeft.setTargetPosition(FrontLeftTarget);
            FrontRight.setTargetPosition(FrontRightTarget);
            RearLeft.setTargetPosition(RearLeftTarget);
            RearRight.setTargetPosition(RearRightTarget);

            FrontLeft.setVelocity(FrLTPS);
            FrontRight.setVelocity(FrRTPS);  //Set Velocity requires DcMotorEx
            RearLeft.setVelocity(ReLTPS);
            RearRight.setVelocity(ReRTPS);

            OdoMethods.turnToHeading(TURN_SPEED, 90.0);               // In Degrees
            OdoMethods.holdHeading(TURN_SPEED, 90.0, 0.5);

            while (opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy() && RearLeft.isBusy() && RearRight.isBusy())) {
                //Keeps Robot from continuing program before all motors have reached target. Not necessarily necessary.
                telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
                telemetry.addData("Frontright", FrontRight.getCurrentPosition());
                telemetry.addData("Rearleft", RearLeft.getCurrentPosition());
                telemetry.addData("Rearright", RearRight.getCurrentPosition());
                ;
                telemetry.update();

                FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                FrontLeftTarget = (int) (44 * TICKS_PER_IN);  //12 is in place of choosen distance in inches.
                FrontRightTarget = (int) (44 * TICKS_PER_IN);
                RearLeftTarget = (int) (44 * TICKS_PER_IN);
                RearRightTarget = (int) (44 * TICKS_PER_IN);
                FrLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // TPS means Ticks Per Second. RPM/60 to convert to seconds
                FrRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // RPM indictes RMP, can be 0-160RPM
                ReLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;
                ReRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;

                FrontLeft.setTargetPosition(FrontLeftTarget);
                FrontRight.setTargetPosition(FrontRightTarget);
                RearLeft.setTargetPosition(RearLeftTarget);
                RearRight.setTargetPosition(RearRightTarget);

                FrontLeft.setVelocity(FrLTPS);
                FrontRight.setVelocity(FrRTPS);  //Set Velocity requires DcMotorEx
                RearLeft.setVelocity(ReLTPS);
                RearRight.setVelocity(ReRTPS);

                OdoMethods.turnToHeading(TURN_SPEED, 90.0);               // In Degrees
                OdoMethods.holdHeading(TURN_SPEED, 90.0, 0.5);

                while (opModeIsActive() && (FrontLeft.isBusy() && FrontRight.isBusy() && RearLeft.isBusy() && RearRight.isBusy())) {
                    //Keeps Robot from continuing program before all motors have reached target. Not necessarily necessary.
                    telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
                    telemetry.addData("Frontright", FrontRight.getCurrentPosition());
                    telemetry.addData("Rearleft", RearLeft.getCurrentPosition());
                    telemetry.addData("Rearright", RearRight.getCurrentPosition());
                    ;
                    telemetry.update();

                    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    RearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    RearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    FrontLeftTarget = (int) (44 * TICKS_PER_IN);  //12 is in place of choosen distance in inches.
                    FrontRightTarget = (int) (44 * TICKS_PER_IN);
                    RearLeftTarget = (int) (44 * TICKS_PER_IN);
                    RearRightTarget = (int) (44 * TICKS_PER_IN);
                    FrLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // TPS means Ticks Per Second. RPM/60 to convert to seconds
                    FrRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;        // RPM indictes RMP, can be 0-160RPM
                    ReLTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;
                    ReRTPS = (RPM / 60) * TICKS_PER_WHEEL_REV;

                    FrontLeft.setTargetPosition(FrontLeftTarget);
                    FrontRight.setTargetPosition(FrontRightTarget);
                    RearLeft.setTargetPosition(RearLeftTarget);
                    RearRight.setTargetPosition(RearRightTarget);

                    FrontLeft.setVelocity(FrLTPS);
                    FrontRight.setVelocity(FrRTPS);  //Set Velocity requires DcMotorEx
                    RearLeft.setVelocity(ReLTPS);
                    RearRight.setVelocity(ReRTPS);

                    OdoMethods.turnToHeading(TURN_SPEED, -90.0);               // In Degrees
                    OdoMethods.holdHeading(TURN_SPEED, -90.0, 0.5);


                    OdoMethods.turnToHeading(TURN_SPEED, 360.0);               // In Degrees
                    OdoMethods.holdHeading(TURN_SPEED, 360.0, 0.5);


                }
            }
        }
    }
}


