package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Pmms", group="TeleOp")
//@Disabled
public class Pmms extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Servo Grabber;

    private Servo pmmA;

    @Override
    public void runOpMode() {
        Grabber = hardwareMap.get(Servo.class, "pmmfloor");
        pmmA = hardwareMap.get(Servo.class, "pmmA");
        while (opModeInInit()) {
            telemetry.addData("Servo Position", pmmA.getPosition());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                pmmA.setDirection(Servo.Direction.FORWARD);
                pmmA.setPosition(0.55);

            }
            // Servos operate 0-180 degrees by a 0-1 metric. This sets servo position to 180 degrees.

            else if (gamepad1.b) {
                pmmA.setDirection(Servo.Direction.FORWARD);
                pmmA.setPosition(0.0);

            }
            else if (gamepad1.x) {
                Grabber.setDirection(Servo.Direction.REVERSE);
                Grabber.setPosition(0.62);

            }
            // Servos operate 0-180 degrees by a 0-1 metric. This sets servo position to 180 degrees.

            else if (gamepad1.y) {
                Grabber.setDirection(Servo.Direction.REVERSE);
                Grabber.setPosition(0.0);

            }
            telemetry.addData("Servo Position", Grabber.getPosition());
            telemetry.update();
            telemetry.addData("Servo Position", pmmA.getPosition());
            telemetry.update();
        }

        }
    }

