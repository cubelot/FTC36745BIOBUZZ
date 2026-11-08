package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "saharsh_auton_homework")

public class saharsh_auton_homework extends OpMode {

    private static final double TARGET_VELOCITY = 1447.5;
    private static final double READY_TOLERANCE = 40;
    private static final long SPINUP_TIMEOUT = 3000;
    private static final long FEED_MS = 4000;
    private static final long DRIVE_TIMEOUT = 8000;

    private static final double LOW_VOLTAGE = 12.25;
    private static final double HIGH_VOLTAGE = 13;

    private static final PIDFCoefficients PIDF_LOW = new PIDFCoefficients(0, 0, 0, 15.5469);
    private static final PIDFCoefficients PIDF_MID = new PIDFCoefficients(44.5, 0, 0, 14.9469);
    private static final PIDFCoefficients PIDF_HIGH = new PIDFCoefficients(25.3, 0, 0, 14.65);

    private static final double[] START = {60, 12, 90};
    private static final double[] CONTROL = {12, 32, 90};
    private static final double[] END = {12, 108, 0};

    private final PoseFactory p = PoseFactory.degrees();
    private final ElapsedTime matchTime = new ElapsedTime();

    private Follower follower;
    private DcMotorEx outtake;
    private DcMotor intake;
    private DcMotor transfer;
    private CRServo leftFeed;
    private CRServo rightFeed;
    private VoltageSensor voltage;

    private boolean red = false;
    private String stage = "init";

    private Pose pose(double[] a) {
        return red ? p.of(144 - a[0], a[1], 180 - a[2]) : p.of(a[0], a[1], a[2]);
    }

    public Path parkPath() {
        return curve(pose(START), pose(CONTROL), pose(END)).linear(pose(START), pose(END));
    }

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        leftFeed = hardwareMap.get(CRServo.class, "left");
        rightFeed = hardwareMap.get(CRServo.class, "right");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) red = false;
        if (gamepad1.b) red = true;

        telemetry.addData("Alliance", red ? "RED" : "BLUE");
        telemetry.addLine("X = BLUE, B = RED");
        telemetry.addData("Battery", "%.2f V", voltage.getVoltage());
        telemetry.update();
    }

    @Override
    public void start() {
        applyPidf();
        follower.setPose(pose(START));
        matchTime.reset();
        schedule(routine());
    }

    private void applyPidf() {
        double v = voltage.getVoltage();
        PIDFCoefficients chosen = v <= LOW_VOLTAGE ? PIDF_LOW : v >= HIGH_VOLTAGE ? PIDF_HIGH : PIDF_MID;
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, chosen);
    }

    private boolean flywheelReady() {
        return Math.abs(outtake.getVelocity() - TARGET_VELOCITY) <= READY_TOLERANCE;
    }

    private Command waitForFlywheel() {
        return Command.build()
                .setStart(() -> {})
                .setDone(this::flywheelReady)
                .setEnd(interrupted -> {});
    }

    private Command routine() {
        return sequential(
                instant(() -> {
                    stage = "spin-up";
                    outtake.setVelocity(TARGET_VELOCITY);
                }),
                race(waitForFlywheel(), waitMs(SPINUP_TIMEOUT)),
                instant(() -> {
                    stage = "feeding";
                    feedOn();
                }),
                waitMs(FEED_MS),
                instant(() -> {
                    stage = "parking";
                    allOff();
                }),
                race(follow(follower, parkPath()), waitMs(DRIVE_TIMEOUT)),
                instant(() -> stage = "done")
        );
    }

    private void feedOn() {
        intake.setPower(1);
        transfer.setPower(1);
        leftFeed.setPower(1);
        rightFeed.setPower(-1);
    }

    private void allOff() {
        outtake.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
        leftFeed.setPower(0);
        rightFeed.setPower(0);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("Stage", stage);
        telemetry.addData("Time", "%.1f s", matchTime.seconds());
        telemetry.addData("Flywheel", "%.0f / %.0f", outtake.getVelocity(), TARGET_VELOCITY);
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (outtake != null) outtake.setPower(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
        if (leftFeed != null) leftFeed.setPower(0);
        if (rightFeed != null) rightFeed.setPower(0);
    }
}
