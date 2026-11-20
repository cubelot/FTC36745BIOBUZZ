package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoTest extends OpMode {
    private Servo flower1;
    private Servo flower2;

    private double val = 0;

    @Override
    public void init() {
        flower1 = hardwareMap.servo.get("flower1");
        flower2 = hardwareMap.servo.get("flower2");

//        flower1.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void loop() {
        if(gamepad1.dpad_left) {
            val += 0.01;
        } else if (gamepad1.dpad_right) {
            val -= 0.01;
        }

        flower1.setPosition(1.2 - val);
        flower2.setPosition(val);
        telemetry.addData("angle (between 1 and 0)", val);

    }
}
