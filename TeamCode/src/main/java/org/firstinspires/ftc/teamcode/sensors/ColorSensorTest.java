package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorSensorTest extends OpMode {
    org.firstinspires.ftc.teamcode.Sensors.TestBenchColor bench = new org.firstinspires.ftc.teamcode.Sensors.TestBenchColor();
    org.firstinspires.ftc.teamcode.Sensors.TestBenchColor.DetectedColor detectedColor;


    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {

        detectedColor = bench.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);

    }
}
