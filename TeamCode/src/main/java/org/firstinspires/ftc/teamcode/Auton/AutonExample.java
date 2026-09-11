package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

@Autonomous
public class AutonExample extends OpMode {

    private Follower follower;

    private final PoseFactory pf = PoseFactory.degrees();//new posefactory thingy; it is supposed to help reduce the length of pose lines

    private final Pose startPose = pf.of(72, 72, 90);
    private final Pose pose1 = pf.of(120, 96, 0);

    private final Pose pose2 = pf.of(72, 24, 56);

    private final Pose pose3 = pf.of(72, 120, 90);

    private final Pose pose4 = new Pose(72, 48);

    private final Pose controlPose2 = new Pose(15, 86);

    private Path startPath() {
        return line(startPose, pose1).linear(startPose, pose1);
    }
    private Path path1(){
        return line(pose1, pose2).reverseTangent();
    }
    private Path path2(){
        return curve(pose2, controlPose2, pose3).linear(pose2, pose3);
    }
    private Path endPath(){
        return line(pose3, pose4).reverseTangent();
    }
    

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start () {

    }

    @Override
    public void loop() {

    }
}