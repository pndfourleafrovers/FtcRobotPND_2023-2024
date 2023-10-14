package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name="Distance", group="Autonomous")
public class DistanceSensorPropDT extends LinearOpMode {
    //Two distance sensors plus the variables their values are given to.
    private DistanceSensor Dis1;
    private DistanceSensor Dis2;

    static double MeasuredDis1;

    static double MeasuredDis2;


    @Override
    public void runOpMode() {
//Initializing Hardware
        Dis1 = hardwareMap.get(DistanceSensor.class, "Dis1");


        Dis2 = hardwareMap.get(DistanceSensor.class, "Dis2");

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName", Dis1.getDeviceName());
            telemetry.addData("deviceName", Dis2.getDeviceName());
            telemetry.addData("range", String.format("%.01f mm", Dis1.getDistance(DistanceUnit.INCH)));
            telemetry.addData("range", String.format("%.01f cm", Dis2.getDistance(DistanceUnit.INCH)));
            telemetry.update();
        }

    }
}
