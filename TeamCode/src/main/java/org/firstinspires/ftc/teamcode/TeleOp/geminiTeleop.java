package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.OpModeStorage;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Saharshs Teleop")
public class geminiTeleop extends OpMode {

    private static final double SLOW_MODE_POWER     = 0.4;
    private static final double FULL_POWER          = 1.0;
    private static final double STRAFE_MULTIPLIER   = 1.1;   // compensates for weaker strafing
    private static final double STICK_DEADBAND      = 0.05;
    private static final double STICK_EXPO          = 0.4;   // 0 = linear, 1 = pure cubic
    private static final double HEADING_RESET_DEG   = 0;     // heading assigned when pressing BACK

    private static final double TICKS_PER_REV       = 28;
    private static final double IDLE_VELOCITY       = 900;   // ticks/sec
    private static final double SHOOT_VELOCITY      = 1470;  // ticks/sec
    private static final double SHOOT_TRIM_STEP     = 25;    // ticks/sec per D-pad press
    private static final double SHOOT_TRIM_LIMIT    = 300;   // max +/- trim
    private static final double READY_TOLERANCE     = 40;    // ticks/sec window counted as "at speed"
    private static final double READY_SETTLE_MS     = 80;    // must stay in window this long
    private static final boolean REQUIRE_READY_TO_FIRE = false;

    private static final double FLYWHEEL_P          = 25;
    private static final double FLYWHEEL_I          = 0;
    private static final double FLYWHEEL_D          = 0;
    private static final double F_SLOPE             = -1.13932;
    private static final double F_INTERCEPT         = 28.46684;

    private static final double VOLT_SAMPLE_MS      = 100;   // how often we actually read the sensor
    private static final double VOLT_FILTER_ALPHA   = 0.2;   // 0..1, higher = less smoothing
    private static final double PIDF_UPDATE_STEP    = 0.05;  // only rewrite PIDF if volts moved this much
    private static final double LOW_BATTERY_VOLTS   = 11.5;

    private static final double INTAKE_POWER        = 1.0;
    private static final double TRANSFER_INTAKE_PWR = 0.8;
    private static final double TRANSFER_FIRE_PWR   = 0.5;
    private static final double GATE_CLOSED         = 0.9;
    private static final double GATE_FIRE           = 0.5;

    private static final double FLOWER_ON           = 0.4;
    private static final double FLOWER_OFF          = 1.0;

    private static final int    RUMBLE_MS           = 250;

    private Follower follower;
    private DcMotorEx outtake;
    private DcMotor intake;
    private DcMotor transfer;
    private VoltageSensor voltage;
    private Servo gate;
    private Servo flower1;

    // Edge detectors (fixes the old mismatched gamepad1/gamepad2 toggle bug)
    private final Edge flowerEdge   = new Edge();
    private final Edge macroEdge    = new Edge();
    private final Edge headingEdge  = new Edge();
    private final Edge trimUpEdge   = new Edge();
    private final Edge trimDownEdge = new Edge();
    private final Edge trimResetEdge = new Edge();

    private boolean flowerOn = false;

    private double shootTrim = 0;
    private double lastTargetVelocity = Double.NaN;
    private boolean flywheelReady = false;
    private boolean wasReady = false;

    private double volts = 12;
    private double pidfAppliedAtVolts = Double.NaN;
    private double feedForward = 0;

    private double lastGatePos = Double.NaN;
    private double lastFlowerPos = Double.NaN;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime voltTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private double loopMs = 0;

    // lifecycle

    @Override
    public void init() {
        // Bulk caching: one hub read per loop instead of one per sensor call
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.create(hardwareMap);

        outtake  = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake   = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        voltage  = hardwareMap.get(VoltageSensor.class, "Control Hub");
        gate     = hardwareMap.get(Servo.class, "gate");
        flower1  = hardwareMap.get(Servo.class, "flower1");

        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Seed voltage + PIDF so the first loop is already compensated
        volts = voltage.getVoltage();
        applyFlywheelPidf();

        telemetry.addLine("Initialized. Waiting for start.");
        telemetry.addData("Auto end pose", OpModeStorage.autonomousEndPose == null ? "NONE (will use 0,0,0)" : "loaded");
        telemetry.addData("Battery", "%.2f V", volts);
        telemetry.update();
    }

    @Override
    public void start() {
        Pose startPose = OpModeStorage.autonomousEndPose;
        follower.setPose(startPose != null ? startPose : new Pose(0, 0, 0));

        loopTimer.reset();
        voltTimer.reset();
        readyTimer.reset();
    }

    @Override
    public void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        updateVoltage();
        handleDrive();
        handleFlywheel();
        handleIntakeTransferGate();
        handleFlower();
        handleMacros();
        updateTelemetry();
    }

    @Override
    public void stop() {
        outtake.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    // subsystems

    private void handleDrive() {
        double power = gamepad1.right_trigger_pressed ? SLOW_MODE_POWER : FULL_POWER;

        double forward = shape(-gamepad1.left_stick_y);
        double strafe  = shape(-gamepad1.left_stick_x);
        double turn    = shape(-gamepad1.right_stick_x);

        Pose pose = follower.pose();

        DrivePowers powers = ManualDrive.fieldCentric(
                forward * power,
                strafe * power * STRAFE_MULTIPLIER,
                turn * power,
                pose.heading()
        );
        ManualDrive.driveOrHold(follower, powers);
        follower.update();

        // Re-zero heading without losing X/Y
        if (headingEdge.pressed(gamepad1.back)) {
            Pose p = follower.pose();
            follower.setPose(new Pose(p.x(), p.y(), Math.toRadians(HEADING_RESET_DEG)));
        }
    }

    private void handleFlywheel() {
        // Shoot-speed trim
        if (trimUpEdge.pressed(gamepad2.dpad_up)) {
            shootTrim += SHOOT_TRIM_STEP;
        }
        if (trimDownEdge.pressed(gamepad2.dpad_down)) {
            shootTrim -= SHOOT_TRIM_STEP;
        }
        if (trimResetEdge.pressed(gamepad2.dpad_right)) {
            shootTrim = 0;
        }
        shootTrim = Range.clip(shootTrim, -SHOOT_TRIM_LIMIT, SHOOT_TRIM_LIMIT);

        boolean shooting = gamepad2.left_trigger_pressed;
        double target = shooting ? (SHOOT_VELOCITY + shootTrim) : IDLE_VELOCITY;

        if (target != lastTargetVelocity) {
            outtake.setVelocity(target);
            lastTargetVelocity = target;
        }

        double measured = outtake.getVelocity();
        boolean inWindow = Math.abs(measured - target) <= READY_TOLERANCE;
        if (!inWindow) {
            readyTimer.reset();
        }
        flywheelReady = inWindow && readyTimer.milliseconds() >= READY_SETTLE_MS;

        // Rumble operator when we hit shoot speed
        if (shooting && flywheelReady && !wasReady) {
            gamepad2.rumble(RUMBLE_MS);
        }
        wasReady = shooting && flywheelReady;
    }

    private void handleIntakeTransferGate() {
        double rev = gamepad2.a ? -1 : 1;

        if (gamepad2.right_bumper) {
            // Intake
            intake.setPower(INTAKE_POWER * rev);
            transfer.setPower(TRANSFER_INTAKE_PWR * rev);
            setGate(GATE_CLOSED);
        } else if (gamepad2.left_bumper && canFire()) {
            // Fire
            intake.setPower(0);
            transfer.setPower(TRANSFER_FIRE_PWR);
            setGate(GATE_FIRE);
        } else {
            // Idle
            intake.setPower(0);
            transfer.setPower(0);
            setGate(GATE_CLOSED);
        }
    }

    private void handleFlower() {
        if (flowerEdge.pressed(gamepad2.y)) {
            flowerOn = !flowerOn;
        }
        setFlower(flowerOn ? FLOWER_ON : FLOWER_OFF);
    }

    private void handleMacros() {
        if (macroEdge.pressed(gamepad1.b)) {
            Macro.turnToHive(follower.pose(), follower);
        }
    }

    private void updateVoltage() {
        if (voltTimer.milliseconds() >= VOLT_SAMPLE_MS) {
            double raw = voltage.getVoltage();
            volts += VOLT_FILTER_ALPHA * (raw - volts);
            voltTimer.reset();
        }

        if (Double.isNaN(pidfAppliedAtVolts)
                || Math.abs(volts - pidfAppliedAtVolts) >= PIDF_UPDATE_STEP) {
            applyFlywheelPidf();
        }
    }

    private void applyFlywheelPidf() {
        feedForward = Math.max(0, F_SLOPE * volts + F_INTERCEPT);
        outtake.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, feedForward)
        );
        pidfAppliedAtVolts = volts;
    }

    // helpers ig

    private boolean canFire() {
        return !REQUIRE_READY_TO_FIRE || flywheelReady || gamepad2.x;
    }

    private void setGate(double pos) {
        if (pos != lastGatePos) {
            gate.setPosition(pos);
            lastGatePos = pos;
        }
    }

    private void setFlower(double pos) {
        if (pos != lastFlowerPos) {
            flower1.setPosition(pos);
            lastFlowerPos = pos;
        }
    }

    private static double shape(double v) {
        double a = Math.abs(v);
        if (a < STICK_DEADBAND) return 0;
        double s = (a - STICK_DEADBAND) / (1 - STICK_DEADBAND);
        double curved = STICK_EXPO * s * s * s + (1 - STICK_EXPO) * s;
        return Math.copySign(curved, v);
    }

    private static class Edge {
        private boolean last = false;

        boolean pressed(boolean now) {
            boolean result = now && !last;
            last = now;
            return result;
        }
    }

   // telemetry
    private void updateTelemetry() {
        Pose pose = follower.pose();
        double velocity = outtake.getVelocity();
        double rpm = (velocity / TICKS_PER_REV) * 60;
        double targetRpm = (lastTargetVelocity / TICKS_PER_REV) * 60;

        if (volts < LOW_BATTERY_VOLTS) {
            telemetry.addLine("!!! LOW BATTERY !!!");
        }

        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("Status", flywheelReady ? "READY" : "spinning...");
        telemetry.addData("RPM / Target", "%.0f / %.0f", rpm, targetRpm);
        telemetry.addData("Velocity (ticks/s)", "%.0f", velocity);
        telemetry.addData("Shoot trim", "%+.0f", shootTrim);
        telemetry.addData("Feedforward", "%.3f", feedForward);

        telemetry.addLine("--- Robot ---");
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Slow mode", gamepad1.right_trigger_pressed);
        telemetry.addData("Flower", flowerOn ? "ON" : "OFF");

        telemetry.addLine("--- System ---");
        telemetry.addData("Battery", "%.2f V", volts);
        telemetry.addData("Loop (ms)", "%.1f", loopMs);
        telemetry.update();
    }
}