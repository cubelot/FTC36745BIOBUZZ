package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Auton1 extends OpMode {

    private Follower follower;

    private final PoseFactory p = PoseFactory.degrees();

    private DcMotor outtake = hardwareMap.get(DcMotor.class, "outtake");
    private DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

    private final Pose startPose = p.of(82, 12, 90);


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
