package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Kickoff Dance")
public class KickoffDance extends LinearOpMode {

    private Follower follower;

    private final Pose startPose = new Pose(72,72,Math.toRadians(90));
    private final Pose toTheLeft_2 = new Pose (48, 72, Math.toRadians(90));
    private final Pose toTheRight_2 = new Pose (96,72,Math.toRadians(90));
    private final Pose takeItBack = new Pose(72, 48, Math.toRadians(90));

    private final Pose toTheLeft_1 = new Pose (60,72,Math.toRadians(90));
    private final Pose toTheRight_1 = new Pose (84, 72, Math.toRadians(90));

    private final Pose hopThisTime_2 = new Pose (72, 84, Math.toRadians(90));
    private final Pose hopThisTime_1 = new Pose (72, 78, Math.toRadians(90));

    private final Pose crissCross_1 = new Pose (48, 84, Math.toRadians(135));
    private final Pose crissCross_2 = new Pose (96, 84, Math.toRadians(45));


    private PathChain toTheLeft;





    private Timer songTimer;


    boolean start = true;
    @Override
    public void runOpMode(){
        //These will run when the OpMode is initiated
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleOpDrive();
        waitForStart();
        songTimer = new Timer();




        int soundID = hardwareMap.appContext.getResources().getIdentifier("chacha", "raw", hardwareMap.appContext.getPackageName());
        while (opModeIsActive()) {
            shimmy(follower, 12, 50);
            follower.update();

        }
        if(opModeIsActive() && start){
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
            start = false;
        }
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
