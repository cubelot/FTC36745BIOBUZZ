package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp
public class TeleOpMode extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    //GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensures positive power moves the robot forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            double y = -this.gamepad1.left_stick_y;
            double x = this.gamepad1.left_stick_x;
            double rx = this.gamepad1.right_stick_x;

            double leftFrontPower = y + x + rx;
            double rightFrontPower = y - x - rx;
            double leftBackPower = y - x + rx;
            double rightBackPower = y + x - rx;

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("LF Power", leftFrontPower);
            telemetry.addData("RF Power", rightFrontPower);
            telemetry.addData("LB Power", leftBackPower);
            telemetry.addData("RB Power", rightBackPower);
            telemetry.update();
            // replaced imu with pinpoint because imu is default and does not work with goBilda pinpoint

            /*
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "GoBildaPinpointDriver");
            GoBildaPinpointDriver.Parameters parameters = new GoBildaPinpointDriver.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
            );
            pinpoint.initialize(parameters);
            */
            waitForStart();
        }
    }
}

/*
public class Sensors {
    private DistanceSensor distance;

    public void init(HardwareMap hwMap) {
        distance = hwMap.get(DistanceSensor.class, "sensorDistance");
    }

    public double getDistance() {
        return distance.getDistance(DistanceUnit.CM);
    }
}

 */


