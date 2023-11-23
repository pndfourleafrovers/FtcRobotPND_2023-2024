package org.firstinspires.ftc.teamcode.Objects;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="Pot Cal", group="TeleOp")
@Disabled
public class Potentiometer extends OpMode {

    // Declare the potentiometer
    AnalogInput potentiometer;

    @Override
    public void init() {
        // Initialize the potentiometer
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    @Override
    public void loop() {
        // Read the raw voltage from the potentiometer
        double voltage = potentiometer.getVoltage();

        // Print the voltage to the driver station
        telemetry.addData("Potentiometer Voltage", voltage);
        telemetry.update();
    }
}
