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
    public DcMotor transfer;
    public DcMotor intake;

    double power = 0;
    double transferPower = 0.6;

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
         }

    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            power += 0.1; }
        if (gamepad1.dpadDownWasPressed()) {
            power -= 0.1; }
        if (gamepad1.dpadRightWasPressed()) {
            power += 0.01; }
        if (gamepad1.dpadLeftWasPressed()) {
            power -= 0.01; }
        if (gamepad1.right_bumper) {
            transfer.setPower(transferPower);
        } else {
            transfer.setPower(0);
        }
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        if (gamepad1.rightTriggerWasPressed()) {
            transferPower += 0.01;
        } else if (gamepad1.leftTriggerWasPressed()) {
            transferPower -= 0.01;
        }
        outtake.setPower(power);
        telemetry.addData("power", power);
        telemetry.addData("transferPower", transferPower);
        telemetry.update(); }

}
