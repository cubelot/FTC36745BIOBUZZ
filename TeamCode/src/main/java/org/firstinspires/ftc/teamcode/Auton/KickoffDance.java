package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class KickoffDance extends LinearOpMode {

    private final Pose startPose = new Pose(72,72,Math.toRadians(90));
    private final Pose toTheLeft = new Pose (48, 72, Math.toRadians(90));
    private final Pose toTheRight = new Pose (96,72,Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void shimmy(Follower follower, int cycles, long speed) {
        for (int i = 0; i < cycles; i++) {
            follower.setTeleOpDrive(0, 0.4, 0, true);
            follower.update();
            sleep(speed);
            follower.setTeleOpDrive(0, -0.4, 0, true);
            follower.update();
            sleep(speed);
        }
        follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();
    }
}