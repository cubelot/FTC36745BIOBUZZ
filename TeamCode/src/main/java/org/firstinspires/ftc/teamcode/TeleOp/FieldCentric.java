

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
            /* Used later for properly rotating Cartesian coordinates (normal coordinate plane)
            mathematically using a certain heading, which tells you the angle difference between
            the current location versus the desired location via rotation by assuming the current
            location is at 0 radians everytime. */
            double theta = odo.getHeading(AngleUnit.DEGREES)-90; //Same as heading but in degrees, used for the following code
            if (theta < 0) {
                theta = 360 + theta;} //makes theta range from the built-in -180 - 180 range to 0 - 360 range
            double pointer = Math.toDegrees(Math.atan2 (ry,rx)); //Converts right joystick motion into degrees (vector),
            // atan2 is the same as taking the arctan for two arguments (inputs).
            if (pointer < 0) {
                pointer = 360 + pointer;} //0-360 range like theta
            double difference = pointer - theta; //Difference checks the closest way to get to your location
            difference = difference - 360.0 * Math.round(difference / 360.0); //Rounds so that it doesn't get stuck between -180 to 180
            double magnitude = Math.hypot(rx,ry); /* Checks how much has the joystick
            been offset from the positon where you are not pushing the joystick*/
            double r; // Makes the rate of turning based on where the desired rotation location is
            if (magnitude>0.2) { //Checks if we are actually actively trying to turn it
                r = Math.signum(difference); //Signum basically takes the sign only, so it can directly go to the directed result
            } else {
                r=0;}
            //The above code basically turns the robot to the direction the right controller is pointing, so it stays relative to the driver

            double rotx = x*Math.cos(-heading)-y*Math.sin(-heading);
            double roty = x*Math.sin(-heading)+y*Math.cos(-heading);
            //Math behind rotation and changing the x and y coordinates accordingly
            double lfPower = roty + rotx + r;
            double lbPower = roty - rotx + r;
            double rfPower = roty - rotx - r;
            double rbPower = roty + rotx - r;
            //Mecanum power formulas, it just works
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            telemetry.addLine("its running");
            telemetry.update();
        }




    }
}