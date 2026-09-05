package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;

import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.50)
            //.forwardZeroPowerAcceleration(-42.343572953089065)
            //.lateralZeroPowerAcceleration(-57.035110445951176)
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.075,0,0.001, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.065, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.2,0,0.03,0.02))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.55, 0, 0.0001, 0.6, 0.02))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.04, 0.05386978922358552, 0.0014252789895499041))
            .centripetalScaling(0)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(65.63270484774095)
            .yVelocity(56.4656540277436);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3)
            .strafePodX(3.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}