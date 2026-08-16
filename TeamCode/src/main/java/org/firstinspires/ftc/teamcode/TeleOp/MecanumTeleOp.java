package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

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

        if(isStopRequested()) return;

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {
            /*double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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
            telemetry.update();*/

            /*
            - you do not need to know motor power.
            - by comp, we will also need two mecanum teleOp files, "TeleOpSingleController" for just one controller
              (specifically for the software testing) and just "TeleOp" for two controllers (the one we will actually be running at comp)
            - you don't need to copy the exact steps from the tutorial, make it easier for yourselves
            - use gm0.org if you need a refresher, do not use it for field centric as the hardware we have is different from the one in gm0
            - pinpoint is NOT an alternative to imu, pinpoint is for tracking the robot pos and heading,
              while imu is for tracking heading and gyro (only time when switching is useful is when detecting heading,
              where pinpoint is superior) */


        }
    }
}



