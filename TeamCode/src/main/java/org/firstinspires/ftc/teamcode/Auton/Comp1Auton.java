package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Comp1Auton extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    private final ElapsedTime shootTime = new ElapsedTime();

    private DcMotor outtake;
    private DcMotor intake;

    private DcMotor transfer;

    private final Pose startPose = p.of(84, 12, 90);
    private final Pose endPose = p.of(12, 108, 90);

    private final Pose control1 = p.of(12, 32, 90);

    public Path path1() {
        return curve(startPose, control1, endPose).constant(startPose);
    }

    @Override
    public void init() {
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

    }

    @Override
    public void start() {
        schedule(shoot());
        follower.follow(path1());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (outtake != null) outtake.setPower(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
    }


    public Command shoot() {
        return Command.build()
                .setStart(() -> {
                    shootTime.reset();
                    outtake.setPower(0.85);
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .setDone(() -> shootTime.milliseconds() >= 10000)
                .setEnd(interrupted -> {
                    outtake.setPower(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                });
    }
}

