package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class AutonExample extends OpMode {

    private Follower follower;

    private final PoseFactory pf = PoseFactory.degrees();

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start () {

    }

    @Override
    public void loop() {

    }
}