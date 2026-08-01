

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Field Centric")
public class FieldCentric extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    //pinpoint
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode(){
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class ,"rb");
        odo =  hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("it works");

        waitForStart();
        if(isStopRequested()) return;


        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double ry = -gamepad1.right_stick_y;
            double rx = gamepad1.right_stick_x;

            odo.update();
            double heading = odo.getHeading(AngleUnit.RADIANS);
            double theta = odo.getHeading(AngleUnit.DEGREES);
            if (theta < 0) {
                theta = 360 + theta;}
            double pointer = Math.toDegrees(Math.atan2(ry,rx));
            if (pointer < 0) {
                pointer = 360 + pointer;}
            double difference = pointer - theta;
            difference = difference - 360.0 * Math.round(difference / 360.0);
            double magnitude = Math.hypot(rx,ry);
            double r;
            if (magnitude>0.2) {
                r = Math.signum(difference); //signum basically takes the sign only
            } else {
                r=0;
            }

            double rotx = x*Math.cos(-heading)-y*Math.sin(-heading);
            double roty = x*Math.sin(-heading)+y*Math.cos(-heading);

            double lfPower = roty + rotx + r;
            double lbPower = roty - rotx + r;
            double rfPower = roty - rotx - r;
            double rbPower = roty + rotx - r;

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            telemetry.addLine("its running");
            telemetry.update();
        }




    }
}