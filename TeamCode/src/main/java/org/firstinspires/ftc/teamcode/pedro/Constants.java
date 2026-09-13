package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");
        c.frontRightName.set("rf");
        c.backLeftName.set("lb");
        c.backRightName.set("rb");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(2.389182894248662);
        c.yPodOffset.set(5.502398633581446);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.1469600906304332);
                Controller secondaryTranslationalForward = Controller.proportional(0.05429782664200167);
                Controller primaryTranslationalLateral = Controller.proportional(0.18349841678032605);
                Controller secondaryTranslationalLateral = Controller.proportional(0.06779776183233116);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0170124208920579));
                c.brake.set(Controller.proportionalFeedforward(0.014460557758249214));

                c.headingFeedback.set(Controller.proportional(3.645593952087772));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.046887637515600045, 0.004292716316657813));

                c.linearBrakeCoefficients.set(Matrix.diag(0.03789650943005836, 0.07453295983807376));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001882040390260141, 7.794927886604673E-4));

                c.maxAchievableForwardVelocity.set(61.329966526368324);
                c.maxAchievableStrafeVelocity.set(52.47781354935544);
                c.naturalForwardDeceleration.set(40.44706958491656);
                c.naturalStrafeDeceleration.set(53.99441935310255);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

}
