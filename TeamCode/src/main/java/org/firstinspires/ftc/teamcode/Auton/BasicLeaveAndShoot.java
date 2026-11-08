package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auton!=Shoot:)uwu-cool")
public class BasicLeaveAndShoot extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    private final ElapsedTime shootTime = new ElapsedTime();


    private final Pose startPose = p.of(12, 132, 0);
    private final Pose endPose = p.of(12, 117, 0);

    private boolean isShootDone = false;



    public Path path1() {
        return line(startPose, endPose).constant(startPose);
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

    }

    @Override
    public void start() {
        schedule(follow(follower, path1()));




    }

    @Override
    public void loop() {
        follower.update();
//        telemetry.addData("Voltage", voltage.getVoltage());
//        telemetry.addData("isShootDone", isShootDone);
//        telemetry.addData("velocity", outtake.getVelocity());
//        telemetry.update();

        Scheduler.execute();
    }

    @Override
    public void stop() {
        Scheduler.reset();
//        if (outtake != null) outtake.setPower(0);
//        if (intake != null) intake.setPower(0);
//        if (transfer != null) transfer.setPower(0);
    }


    /*public Command shoot() {
        return Command.build()
                .setStart(() -> {
                    shootTime.reset();
                    outtake.setVelocity(1440);
                    if(shootTime.milliseconds() >= 3000) {

                    transfer.setPower(0.55);
                    }
                })
                .setDone(() -> shootTime.milliseconds() >= 7000)
                .setEnd(interrupted -> {
                    outtake.setVelocity(0);
                    intake.setPower(0);
                    transfer.setPower(0);
                    isShootDone = true;
                });
    }*/

    }



