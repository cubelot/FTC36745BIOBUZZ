package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rightBack;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");

            /*
            telemetry.addData("LF Power", leftFrontPower);
            telemetry.addData("RF Power", rightFrontPower);
            telemetry.addData("LB Power", leftBackPower);
            telemetry.addData("RB Power", rightBackPower);
             */
            telemetry.update();

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



