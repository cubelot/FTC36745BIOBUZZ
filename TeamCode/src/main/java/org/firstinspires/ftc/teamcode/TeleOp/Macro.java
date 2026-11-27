package org.firstinspires.ftc.teamcode.TeleOp;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;

public class Macro {
    public static void turnToHive(Pose currentPose, Follower follower) {
        if(currentPose.y() > 72) {
            double targetHeading = 1 - (Math.atan2(88 - currentPose.y(), 58 - currentPose.x()));
            follower.hold(new Pose(currentPose.x(), currentPose.y(), targetHeading));

        } else if(currentPose.y() < 72) {
            double targetHeading = 1 - (Math.atan2(55 - currentPose.y(), 58 - currentPose.x()));
            follower.hold(new Pose(currentPose.x(), currentPose.y(), targetHeading));
        }
    }

}