package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoTest extends OpMode {
    private Servo flowerArm1;
    private Servo flowerArm2;

    private double val = 0;

    @Override
    public void init() {
        flowerArm1 = hardwareMap.servo.get("flowerArm1");
        flowerArm2 = hardwareMap.servo.get("flowerArm2");

        flowerArm1.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void loop() {
        if(gamepad1.dpad_left) {
            val += 0.01;
        } else if (gamepad1.dpad_right) {
            val -= 0.01;
        }

        flowerArm1.setPosition(1 - val);
        flowerArm2.setPosition(val);
        telemetry.addData("angle (between 1 and 0)", val);

    }
}
