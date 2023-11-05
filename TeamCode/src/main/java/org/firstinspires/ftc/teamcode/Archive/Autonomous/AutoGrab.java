package org.firstinspires.ftc.teamcode.Archive.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//Figure out if the pmmfloor starts clasped or not. If it is not, you will need to start it at the beggining of the match

@Autonomous(name="AutoGrab", group="Autonomous")
@Disabled
public class AutoGrab extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo Grabber;

    @Override
    public void runOpMode() {
//Becomes clasped
        Grabber = hardwareMap.get(Servo.class, "pmmfloor");

        waitForStart();

        Grabber.setPosition(0.6); //Check to see what the effective value of this is.

        telemetry.addData("Servo Position", Grabber.getPosition());
        telemetry.update();
//Becomes released
        Grabber.setPosition(0);

        telemetry.addData("Servo Position", Grabber.getPosition());
        telemetry.update();
    }
}
