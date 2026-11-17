package org.firstinspires.ftc.teamcode.TeleOp;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;

public class Macro {
    public static void turnToHive(Follower follower) {
        Pose currentPose = follower.pose();
        if(currentPose.y() > 72) {
            double targetHeading = Math.atan2(84 - currentPose.y(), 60 - currentPose.x());
            follower.hold(new Pose(currentPose.x(), currentPose.y(), targetHeading));

        } else if(currentPose.y() < 72) {
            double targetHeading = Math.atan2(60 - currentPose.y(), 60 - currentPose.x());
            follower.hold(new Pose(currentPose.x(), currentPose.y(), targetHeading));
        }
    }

    public static void flowerArm(Servo servo) {
    servo.setPosition(0.05);
    }
    public static void unflowerArm(Servo servo) {
    servo.setPosition(0.65);
    }
}