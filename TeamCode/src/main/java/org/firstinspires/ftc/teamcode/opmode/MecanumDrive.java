package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp
public class MecanumDrive {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    public init(HardwareMap hwMap) {
        frontRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hwMap.get(DcMotor.class, "back_right_motor");
        frontLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


















        frontRightMotor.setPower(pivot + (-vertical + horizontal));
        backRightMotor.setPower(pivot + (-vertical - horizontal));
        frontLeftMotor.setPower(pivot + (-vertical - horizontal));
        frontRightMotor.setPower(pivot + (-vertical + horizontal));

























        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        vertical = gamepad1.left_stick_y; // Test if it has to be neg.
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;

        frontRightMotor.setPower(pivot + (-vertical + horizontal));
        backRightMotor.setPower(pivot + (-vertical - horizontal));
        frontLeftMotor.setPower(pivot + (-vertical - horizontal));
        RFMotor.setPower(pivot + (-vertical + horizontal));

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void init() {
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

    }
    @Override
    public class init_loop() {

    }

    @Override
    public void loop() {


    }

}
