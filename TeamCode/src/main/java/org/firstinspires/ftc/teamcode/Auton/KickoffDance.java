package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class KickoffDance extends LinearOpMode {

    private final Pose startPose1 = new Pose(72,72,Math.toRadians(90));
    private final Pose toTheLeft_2 = new Pose (48, 72, Math.toRadians(90));
    private final Pose toTheRight_2 = new Pose (96,72,Math.toRadians(90));
    private final Pose takeItBack = new Pose(72, 48, Math.toRadians(90));

    private final Pose toTheLeft_1 = new Pose (60,72,Math.toRadians(90));
    private final Pose toTheRight_1 = new Pose (84, 72, Math.toRadians(90));

    private final Pose hopThisTime_2 = new Pose (72, 84, Math.toRadians(90));
    private final Pose hopThisTime_1 = new Pose (72, 78, Math.toRadians(90));

    private final Pose crissCross_1 = new Pose (48, 84, Math.toRadians(135));
    private final Pose crissCross_2 = new Pose (96, 84, Math.toRadians(45));

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        int soundID = hardwareMap.appContext.getResources()
                .getIdentifier("your_song", "raw", hardwareMap.appContext.getPackageName());

        // 1. Initialize and wait for match start
        waitForStart();

        if (opModeIsActive()) {
            // 2. Start the song immediately when Autonomous begins
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
        }

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
