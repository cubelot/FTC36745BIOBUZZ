package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class FlywheelPowerTest extends OpMode {

    public DcMotor outtake;

    double power = 0;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        outtake.setPower(power); }

    public void loop() {
        if (gamepad1.dpad_up) {
            power += 0.1; }
        if (gamepad1.dpad_down) {
            power -= 0.1; }
        if (gamepad1.dpad_right) {
            power += 0.01; }
        if (gamepad1.dpad_left) {
            power -= 0.01; }
        telemetry.addData("power", power);
        telemetry.update(); }

}
