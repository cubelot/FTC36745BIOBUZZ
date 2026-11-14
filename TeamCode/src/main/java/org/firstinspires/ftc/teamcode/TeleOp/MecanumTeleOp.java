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
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@TeleOp (name = "Mecanum TeleOp")
public class ThreeWheelOdometry extends OpMode {

    // ---- Tune these for your robot ----
    private static final double TICKS_PER_REV = 2000;     // e.g. REV Through Bore / goBILDA
    private static final double WHEEL_RADIUS_IN = 0.944;  // 48 mm wheel
    private static final double TRACK_WIDTH_IN = 12.0;    // distance between left and right wheels
    private static final double FORWARD_OFFSET_IN = 4.0;  // center wheel offset from robot center

    private static final double INCHES_PER_TICK =
            (2.0 * Math.PI * WHEEL_RADIUS_IN) / TICKS_PER_REV;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Odometry encoders (plugged into motor/encoder ports)
    private DcMotorEx leftEncoder, rightEncoder, centerEncoder;

    // Pose state
    private double xPos = 0;       // inches
    private double yPos = 0;       // inches
    private double heading = 0;    // radians

    // Previous encoder readings (ticks)
    private int prevLeftPos = 0;
    private int prevRightPos = 0;
    private int prevCenterPos = 0;

    @Override
    public void init() {
        // Use the names from your robot configuration
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder = hardwareMap.get(DcMotorEx.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "rightEncoder");
        centerEncoder = hardwareMap.get(DcMotorEx.class, "centerEncoder");

        // Reverse as needed so that:
        //   - left/right both increase when driving forward
        //   - center increases when strafing left
        // leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx enc : new DcMotorEx[]{leftEncoder, rightEncoder, centerEncoder}) {
            enc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            enc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateOdometry();
        driveMecanum();

        telemetry.addData("x (in)", "%.2f", xPos);
        telemetry.addData("y (in)", "%.2f", yPos);
        telemetry.addData("heading (deg)", "%.2f", Math.toDegrees(heading));
        telemetry.update();
    }

    private void driveMecanum() {
        double y = -gamepad1.left_stick_y;   // forward
        double x = gamepad1.left_stick_x * 1.1; // strafe (counteract imperfect strafing)
        double rx = gamepad1.right_stick_x;  // turn

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    private void updateOdometry() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int centerPos = centerEncoder.getCurrentPosition();

        // Change in each encoder, converted to inches
        double deltaLeftPos = (leftPos - prevLeftPos) * INCHES_PER_TICK;
        double deltaRightPos = (rightPos - prevRightPos) * INCHES_PER_TICK;
        double deltaCenterPos = (centerPos - prevCenterPos) * INCHES_PER_TICK;

        double phi = (deltaLeftPos - deltaRightPos) / TRACK_WIDTH_IN;
        double deltaMiddlePos = (deltaLeftPos + deltaRightPos) / 2.0;
        double deltaPerpPos = deltaCenterPos - FORWARD_OFFSET_IN * phi;

        double deltaX = deltaMiddlePos * Math.cos(heading) - deltaPerpPos * Math.sin(heading);
        double deltaY = deltaMiddlePos * Math.sin(heading) + deltaPerpPos * Math.cos(heading);

        xPos += deltaX;
        yPos += deltaY;
        heading += phi;

        prevLeftPos = leftPos;
        prevRightPos = rightPos;
        prevCenterPos = centerPos;
    }
}

private GoBildaPinpointDriver odo;


public class MecanumTeleOp extends LinearOpMode {
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

        odo =  hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(1, -6, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

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


            if (gamepad1.dpad_left && !leftBumper) {
                power += 0.01;

            } else if (gamepad1.dpad_right && !rightBumper) {
                power -= 0.01;
            }
            vChange = 13.5/ Math.max(volts, 13.5);

            leftBumper = gamepad1.dpad_left;
            rightBumper = gamepad1.dpad_right;

            if (gamepad1.left_trigger_pressed) {

                outtake.setVelocity(1440);
            } else {
                outtake.setPower(0);
            }
            if (gamepad1.right_trigger_pressed) {
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
            if (gamepad1.a) {
                rev = -1;
            } else {
                rev = 1;
            }
            if (gamepad1.right_bumper) {

                intake.setPower(1*rev);
                left.setPower(1*rev);
                right.setPower(-1*rev);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
                left.setPower(0);
                right.setPower(0);

            }
            if (gamepad1.b) {
                gate.setPosition(0.2);
            } else {
                gate.setPosition(0.5);
            }

            if (gamepad1.left_bumper) {
                transfer.setPower(0.55 * rev);

            } else {
                transfer.setPower(0);
            }
//
//            if(voltage.getVoltage() < 12.25) {
//                isBelowVoltage = true;
//            } else {
//                isBelowVoltage = false;
//            }

            double rpm = (outtake.getVelocity() / 28) * 60;


            odo.update();
            double heading = odo.getHeading(AngleUnit.RADIANS);


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
