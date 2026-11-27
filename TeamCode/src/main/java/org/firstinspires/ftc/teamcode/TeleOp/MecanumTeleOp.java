package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Mecanum TeleOp")
public class MecanumTeleOp extends LinearOpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotorEx outtake;
    DcMotor intake;
    DcMotor transfer;

    Servo flower1;
    Servo flower2;
    Servo gate;
    private VoltageSensor voltage;

    // Pinpoint computes the odometry (x, y, heading) for us
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        gate = hardwareMap.get(Servo.class, "gate");
        flower1 = hardwareMap.get(Servo.class, "flower1");
//        flower2 = hardwareMap.get(Servo.class, "flower2");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(1, -6, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(30, 0, 0, 13.8);
//        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double power = 0.75;
        double mPower = 1;
        int rev = 1;

        boolean flower = false;
        boolean flowerToggle = false;
        boolean leftBumper = false;
        boolean rightBumper = false;
        boolean isBelowVoltage = false;
        String mode = "Fast";

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ---- Mecanum drive ----
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

            // ---- Power tweaking ----
            if (gamepad1.dpad_left && !leftBumper) {
                power += 0.01;
            } else if (gamepad1.dpad_right && !rightBumper) {
                power -= 0.01;
            }
            leftBumper = gamepad1.dpad_left;
            rightBumper = gamepad1.dpad_right;

            // Read voltage every loop (it was only read once before)
            double volts = voltage.getVoltage();
            //linearly
            outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25, 0 ,0, -1.13932*volts + 28.46684));
            //powerly
//            outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25, 0 ,0, 208.95938 * Math.pow(volts,-1.06414)));
            // ---- Outtake / slow mode ----
            if (gamepad1.left_trigger_pressed) {
                outtake.setVelocity(1470);
            } else {
                outtake.setVelocity(900);
            }
            if (gamepad1.right_trigger_pressed) {
                mPower = 0.5;
            } else {
                mPower = 1;
            }

            /*
            controls:
            A: reverse
            Y: Flower

             */
            rev = gamepad1.a ? -1 : 1;
            if(gamepad1.y && !flower){
                flowerToggle = !flowerToggle;
            }

            if (flowerToggle) {
//
                flower1.setPosition(0.4);
//                flower2.setPosition(0.6);

            } else   {
                flower1.setPosition(1);
//                flower2.setPosition(0);
            }
            flower = gamepad1.y;
            if (gamepad1.right_bumper) {
                intake.setPower(1 * rev);
                transfer.setPower(0.8 * rev);
                gate.setPosition(0.5);
            } else if (gamepad1.left_bumper) {
                transfer.setPower(0.5);
                gate.setPosition(0.9);
                intake.setPower(0);
            } else {
                    intake.setPower(0);
                    transfer.setPower(0);
                    gate.setPosition(0.5);
            }


//            gate.setPosition(gamepad1.b ? 0.5 : 0.9);



            double rpm = (outtake.getVelocity() / 28) * 60;

            // ---- Odometry from Pinpoint ----
            odo.update();
            double xPos = odo.getPosX(DistanceUnit.INCH);
            double yPos = odo.getPosY(DistanceUnit.INCH);
            double heading = odo.getHeading(AngleUnit.RADIANS);

            telemetry.addLine("its running");
            telemetry.addData("power", power);
            telemetry.addData("isBelowVoltage", isBelowVoltage);
            telemetry.addData("volts", volts);
            telemetry.addData("velocity", outtake.getVelocity());
            telemetry.addData("rpm", rpm);
            telemetry.addData("Mode", mode);
            telemetry.addData("x (in)", "%.2f", xPos);
            telemetry.addData("y (in)", "%.2f", yPos);
            telemetry.addData("heading (deg)", "%.2f", Math.toDegrees(heading));
            telemetry.update();
        }
    }
}