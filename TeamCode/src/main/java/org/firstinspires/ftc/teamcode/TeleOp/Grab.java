package org.firstinspires.ftc.teamcode.TeleOp;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Grab", group="TeleOp")
@Disabled
public class Grab extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo Grabber;

    @Override
    public void runOpMode() {

        Grabber = hardwareMap.get(Servo.class, "pmmfloor");

    waitForStart();

    while (opModeIsActive()){

        if(gamepad1.a)
            Grabber.setPosition(1);
        // Servos operate 0-180 degrees by a 0-1 metric. This sets servo position to 180 degrees.
        else if(gamepad1.b)
            Grabber.setPosition(0.0);
    }
    telemetry.addData("Servo Position", Grabber.getPosition());
    telemetry.update();
    }
    // Should arm movement go with this part, or be attached to a separate input?


//Maybe use && for adding more functions for buttons. Not too many though, that'd just get confusing.

//find out how to make this a method we can call upon, kinda forgot.
}
/*
    // run until the end of the match (driver presses STOP)
    double tgtPower = 0;
while (opModeIsActive()) {
        tgtPower = -this.gamepad1.left_stick_y;
        motorTest.setPower(tgtPower);
        // check to see if we need to move the servo.
        if(gamepad1.y) {
        // move to 0 degrees.
        servoTest.setPosition(0);
        } else if (gamepad1.x || gamepad1.b) {
        // move to 90 degrees.
        servoTest.setPosition(0.5);
        } else if (gamepad1.a) {
        // move to 180 degrees.
        servoTest.setPosition(1);
        }
        telemetry.addData("Servo Position", servoTest.getPosition());
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", motorTest.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();

        }

        This is some example code. Maybe we have to do something with the power? Not really sure what they are doing there. a
 */