package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous
public class AutonExample extends OpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose point1 = poseFactory.of(56, 36.27, 270);
    private final Pose point2 = poseFactory.of(13.3302, 46.8445, 180);
    private final Pose point3 = poseFactory.of(56, 36, 270);
    private final Pose point4 = poseFactory.of(10.9714, 15.5239, -90.6702);
    private final Pose point4Control1 = poseFactory.of(10.6212, 4.9246, 0);
    private final Pose point5Start = poseFactory.of(10.9714, 15.5239, 0);
    private final Pose point5 = poseFactory.of(56, 36, 270);

    public Path path1() {
        return line(start, point1).constant(point1);
    }

    public Path path2() {

        return line(point1, point2).linear(point2, point1);
    }

    public Path path3() {
        return line(point2, point3).linear(point3, point2);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4).reverseTangent();
    }

    public Path path5() {
        return line(point5Start, point5).linear(point5, point5Start);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addLine("running fine :)");
        telemetry.addLine("B-K, Have it your way, you rule!");
        telemetry.update();
    }
}