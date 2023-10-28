package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="pmmA", group="TeleOp")
//@Disabled
public class pmmA extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo pmmA;

    @Override
    public void runOpMode() {

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
                pmmA.setPosition(0.);

            }
            telemetry.addData("Servo Position", pmmA.getPosition());
            telemetry.update();
        }
    }
}
