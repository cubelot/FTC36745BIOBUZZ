package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class TwoControllerMecanumTeleOp extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotorEx outtake;
    DcMotor intake;
    DcMotor transfer;

    CRServo left;
    CRServo right;
    Servo gate;
    private VoltageSensor voltage;


    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        gate = hardwareMap.get(Servo.class, "gate");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(44.5, 0, 0, 14.9469);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(0, 0, 0, 15.5469);
        PIDFCoefficients pidfCoefficients3 = new PIDFCoefficients(25.3, 0, 0, 14.65);
        if(voltage.getVoltage() <= 12.25) {
            outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);
        } else if (voltage.getVoltage() >= 13){
            outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients3);
        } else {
            outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        }
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power;
        double mPower;
        double volts = voltage.getVoltage();
        double vChange = 1;


        int rev = 1;
        mPower = 1;

        power = 0.75;
        boolean isSlowMode = false;


        boolean leftBumper = false;
        boolean rightBumper = false;
        boolean isBelowVoltage = false;
        String mode = "Fast";

        waitForStart();

        if(isStopRequested()) return;



        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * mPower;
            double backLeftPower = (y - x + rx) / denominator * mPower;
            double frontRightPower = (y - x - rx) / denominator * mPower;
            double backRightPower = (y + x - rx) / denominator * mPower;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);


            vChange = 13.5/ Math.max(volts, 13.5);

            leftBumper = gamepad1.dpad_left;
            rightBumper = gamepad1.dpad_right;

            if (gamepad2.left_trigger_pressed) {

                outtake.setVelocity(1440);
            } else {
                outtake.setPower(0);
            }
            if (gamepad2.right_trigger_pressed) {
                mPower = 0.4;
            } else {
                mPower = 1;
            }


            /*
            controls:
            A: reverse mode
            right button: intake
            b: open gate
            left button: transfer
            Left Trigger: Outtake only
            Right Trigger: slow transfer
             */
            if (gamepad2.a) {
                rev = -1;
            } else {
                rev = 1;
            }
            if (gamepad2.right_bumper) {

                intake.setPower(1*rev);
                left.setPower(1*rev);
                right.setPower(-1*rev);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
                left.setPower(0);
                right.setPower(0);

            }
            if (gamepad2.b) {
                gate.setPosition(0.2);
            } else {
                gate.setPosition(0.5);
            }

            if (gamepad2.left_bumper) {
                transfer.setPower(0.55 * rev);

            }

            double rpm = (outtake.getVelocity() / 28) * 60;


            telemetry.addLine("its running");
            telemetry.addData("power", power);
            telemetry.addData("isBelowVoltage", isBelowVoltage);
            telemetry.addData("volts", volts);
            telemetry.addData("velocity", outtake.getVelocity());
            telemetry.addData("rpm", rpm);
            telemetry.addData("Mode", mode);
            telemetry.update();
        }
    }
}
