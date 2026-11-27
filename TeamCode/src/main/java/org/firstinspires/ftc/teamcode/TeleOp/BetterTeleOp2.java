package org.firstinspires.ftc.teamcode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Better TeleOp")
public class BetterTeleOp2 extends OpMode {
    private Follower follower;
    DcMotorEx outtake;
    DcMotor intake;
    DcMotor transfer;
    VoltageSensor voltage;
    Servo gate;
    Servo flower1;
    double mPower = 1;
    int rev = 1;

    boolean flower = false;
    boolean flowerToggle = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");
        gate = hardwareMap.get(Servo.class, "gate");
        flower1 = hardwareMap.get(Servo.class, "flower1");
    }

    @Override
    public void start() {
        follower.setPose(OpModeStorage.autonomousEndPose);
    }

    @Override
    public void loop() {

        //Driving
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * mPower,
                gamepad1.left_stick_x * mPower,
                gamepad1.right_stick_x * mPower,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();



        // voltage
        double volts = voltage.getVoltage();

        double feedForward = -1.13932*volts + 28.46684;
//        double feedForward = 208.95938 * Math.pow(volts,-1.06414);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25, 0 ,0, feedForward));

        // ---- Outtake / slow mode ----
        if (gamepad2.left_trigger_pressed) {
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
            Left Trigger: Outtake motor
            Right Trigger: Slow Mode
            Left Bumper: Fire
            Right Bumper: Intake

             */
        rev = gamepad2.a ? -1 : 1;
        if(gamepad2.y && !flower){
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
        if (gamepad2.right_bumper) {
            intake.setPower(1 * rev);
            transfer.setPower(0.8 * rev);
            gate.setPosition(0.5);
        } else if (gamepad2.left_bumper) {
            transfer.setPower(0.5);
            gate.setPosition(0.9);
            intake.setPower(0);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
            gate.setPosition(0.5);
        }

        double rpm = (outtake.getVelocity() / 28) * 60;


        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("volts", volts);
        telemetry.addData("velocity", outtake.getVelocity());
        telemetry.addData("rpm", rpm);
        telemetry.addData("Feedforward", feedForward);
        telemetry.update();
    }
}