package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/**
 * Controls (gamepad1)
 *   Left stick ........ drive (forward / strafe)
 *   Right stick X ..... turn
 *   Right trigger ..... slow mode (hold)
 *   Y ................. toggle field-centric / robot-centric
 *   Options ........... zero field-centric heading (face the robot "forward" first)
 *   Right bumper ...... intake
 *   Left bumper ....... transfer
 *   A ................. reverse intake / transfer (hold)
 *   B ................. open gate (hold)
 *   Left trigger ...... spin up outtake (hold). Controller rumbles when at speed.
 *   D-pad up / down ... outtake target velocity +/- 20 ticks/s
 */
@TeleOp(name = "Saharsh TeleOp Test")
public class saharshThingy extends LinearOpMode {

    // ================= Tunables =================
    // Input shaping
    private static final double DEADZONE = 0.05;
    private static final double EXPO = 0.5;            // 0 = linear, 1 = full cubic
    private static final double STRAFE_MULT = 1.1;     // counteracts imperfect strafing
    private static final double SLOW_MULT = 0.4;

    // Slew rates, in "full power per second"
    private static final double DRIVE_ACCEL = 6.0;     // speeding up
    private static final double DRIVE_DECEL = 14.0;    // slowing down (kept snappy for control)
    private static final double TURN_ACCEL = 10.0;
    private static final double TURN_DECEL = 20.0;

    // Outtake
    private static final double OUTTAKE_START_VEL = 1440;  // ticks/s
    private static final double OUTTAKE_VEL_STEP = 20;
    private static final double OUTTAKE_READY_TOL = 40;    // ticks/s
    private static final double OUTTAKE_TICKS_PER_REV = 28;

    // Voltage tiers for the PIDF sets (with hysteresis so they don't flap)
    private static final double LOW_VOLT = 12.25;
    private static final double HIGH_VOLT = 13.0;
    private static final double VOLT_HYST = 0.1;
    private static final double VOLT_CHECK_SEC = 0.5;

    // Servo positions
    private static final double GATE_OPEN = 0.2;
    private static final double GATE_CLOSED = 0.5;

    // Mechanism powers
    private static final double TRANSFER_POWER = 0.55;
    // ============================================

    // Drive
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;

    // Mechanisms
    private DcMotorEx outtake;
    private DcMotor intake, transfer;
    private CRServo leftCr, rightCr;
    private Servo gate;

    // Sensors
    private VoltageSensor voltage;
    private GoBildaPinpointDriver odo;

    // PIDF sets
    private final PIDFCoefficients pidfMid = new PIDFCoefficients(44.5, 0, 0, 14.9469);
    private final PIDFCoefficients pidfLow = new PIDFCoefficients(0, 0, 0, 15.5469);
    private final PIDFCoefficients pidfHigh = new PIDFCoefficients(25.3, 0, 0, 14.65);
    private int pidfTier = -1; // 0 = low, 1 = mid, 2 = high

    // Command caches (avoid sending identical commands to the hub every loop)
    private double lastLf = Double.NaN, lastRf = Double.NaN, lastLb = Double.NaN, lastRb = Double.NaN;
    private double lastIntake = Double.NaN, lastTransfer = Double.NaN, lastCrLeft = Double.NaN, lastCrRight = Double.NaN;
    private double lastGate = Double.NaN;
    private double lastOuttakeCmd = Double.NaN;

    // Slewed drive commands
    private double cmdX = 0, cmdY = 0, cmdRx = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Bulk reads: one hub read per loop instead of one per sensor ----
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ---- Hardware map ----
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");
        leftCr = hardwareMap.get(CRServo.class, "left");
        rightCr = hardwareMap.get(CRServo.class, "right");
        gate = hardwareMap.get(Servo.class, "gate");
        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(1, -6, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        // ---- Motor setup ----
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx m : new DcMotorEx[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the flywheel coast down
        applyPidfForVoltage(voltage.getVoltage());

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---- State ----
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime voltTimer = new ElapsedTime();
        double volts = voltage.getVoltage();

        double outtakeTarget = OUTTAKE_START_VEL;
        boolean fieldCentric = true;
        double headingOffset = 0;

        boolean prevUp = false, prevDown = false, prevY = false, prevOptions = false;
        boolean wasReady = false;

        while (opModeIsActive()) {
            // New bulk read for this loop
            for (LynxModule hub : hubs) hub.clearBulkCache();

            double dt = Math.max(loopTimer.seconds(), 1e-3);
            loopTimer.reset();

            // ---- Odometry ----
            odo.update();
            double rawHeading = odo.getHeading(AngleUnit.RADIANS);
            double heading = normalizeAngle(rawHeading - headingOffset);

            // ---- Voltage check (infrequent; PIDF writes are hub commands) ----
            if (voltTimer.seconds() >= VOLT_CHECK_SEC) {
                voltTimer.reset();
                volts = voltage.getVoltage();
                applyPidfForVoltage(volts);
            }

            // ---- Button edges ----
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean yBtn = gamepad1.y, options = gamepad1.options;

            if (up && !prevUp) outtakeTarget += OUTTAKE_VEL_STEP;
            if (down && !prevDown) outtakeTarget -= OUTTAKE_VEL_STEP;
            outtakeTarget = Range.clip(outtakeTarget, 0, 2400);
            if (yBtn && !prevY) fieldCentric = !fieldCentric;
            if (options && !prevOptions) headingOffset = rawHeading;

            prevUp = up;
            prevDown = down;
            prevY = yBtn;
            prevOptions = options;

            // ---- Drive input shaping ----
            double inY = shape(-gamepad1.left_stick_y);
            double inX = shape(gamepad1.left_stick_x) * STRAFE_MULT;
            double inRx = shape(gamepad1.right_stick_x);

            if (fieldCentric) {
                double cos = Math.cos(-heading);
                double sin = Math.sin(-heading);
                double rotX = inX * cos - inY * sin;
                double rotY = inX * sin + inY * cos;
                inX = rotX;
                inY = rotY;
            }

            double speed = gamepad1.right_trigger_pressed ? SLOW_MULT : 1.0;
            inX *= speed;
            inY *= speed;
            inRx *= speed;

            // ---- Slew limiting (smooth accel, snappy decel) ----
            cmdX = slew(cmdX, inX, DRIVE_ACCEL, DRIVE_DECEL, dt);
            cmdY = slew(cmdY, inY, DRIVE_ACCEL, DRIVE_DECEL, dt);
            cmdRx = slew(cmdRx, inRx, TURN_ACCEL, TURN_DECEL, dt);

            double denom = Math.max(Math.abs(cmdY) + Math.abs(cmdX) + Math.abs(cmdRx), 1.0);
            setDrive(
                    (cmdY + cmdX + cmdRx) / denom,   // LF
                    (cmdY - cmdX - cmdRx) / denom,   // RF
                    (cmdY - cmdX + cmdRx) / denom,   // LB
                    (cmdY + cmdX - cmdRx) / denom);  // RB

            // ---- Mechanisms ----
            int rev = gamepad1.a ? -1 : 1;

            setIntake(gamepad1.right_bumper ? rev : 0,
                    gamepad1.right_bumper ? rev : 0,
                    gamepad1.right_bumper ? -rev : 0);

            setTransfer(gamepad1.left_bumper ? TRANSFER_POWER * rev : 0);
            setGate(gamepad1.b ? GATE_OPEN : GATE_CLOSED);

            // ---- Outtake ----
            boolean outtakeOn = gamepad1.left_trigger_pressed;
            double outtakeVel = outtake.getVelocity();
            if (outtakeOn) {
                if (lastOuttakeCmd != outtakeTarget) {
                    outtake.setVelocity(outtakeTarget);
                    lastOuttakeCmd = outtakeTarget;
                }
            } else if (lastOuttakeCmd != 0) {
                outtake.setPower(0);
                lastOuttakeCmd = 0;
            }

            boolean ready = outtakeOn && Math.abs(outtakeVel - outtakeTarget) <= OUTTAKE_READY_TOL;
            if (ready && !wasReady) gamepad1.rumble(200);
            wasReady = ready;

            double rpm = (outtakeVel / OUTTAKE_TICKS_PER_REV) * 60.0;

            // ---- Telemetry ----
            telemetry.addData("Loop (Hz)", "%.0f", 1.0 / dt);
            telemetry.addData("Drive mode", fieldCentric ? "Field-centric" : "Robot-centric");
            telemetry.addData("Slow mode", gamepad1.right_trigger_pressed);
            telemetry.addData("Volts", "%.2f (PIDF tier %d)", volts, pidfTier);
            telemetry.addData("Outtake target / actual", "%.0f / %.0f", outtakeTarget, outtakeVel);
            telemetry.addData("Outtake RPM", "%.0f", rpm);
            telemetry.addData("Outtake READY", ready);
            telemetry.addData("x (in)", "%.2f", odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("y (in)", "%.2f", odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.update();
        }

        // Safe shutdown
        setDrive(0, 0, 0, 0);
        outtake.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
        leftCr.setPower(0);
        rightCr.setPower(0);
    }

    // ================= Helpers =================

    /** Deadzone (rescaled so output still reaches 1.0) plus an expo curve for fine low-speed control. */
    private static double shape(double v) {
        double a = Math.abs(v);
        if (a < DEADZONE) return 0;
        a = (a - DEADZONE) / (1.0 - DEADZONE);
        a = (1.0 - EXPO) * a + EXPO * a * a * a;
        return Math.copySign(a, v);
    }

    /** Move current toward target, limited by accel (growing magnitude) or decel (shrinking). */
    private static double slew(double current, double target, double accel, double decel, double dt) {
        double rate = Math.abs(target) > Math.abs(current) ? accel : decel;
        double step = rate * dt;
        return current + Range.clip(target - current, -step, step);
    }

    private static double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    /** Only talk to the hub when a value actually changes. */
    private static boolean changed(double a, double b) {
        return Double.isNaN(a) || Math.abs(a - b) > 0.001;
    }

    private void setDrive(double lf, double rf, double lb, double rb) {
        if (changed(lastLf, lf)) { leftFront.setPower(lf); lastLf = lf; }
        if (changed(lastRf, rf)) { rightFront.setPower(rf); lastRf = rf; }
        if (changed(lastLb, lb)) { leftBack.setPower(lb); lastLb = lb; }
        if (changed(lastRb, rb)) { rightBack.setPower(rb); lastRb = rb; }
    }

    private void setIntake(double intakePower, double leftPower, double rightPower) {
        if (changed(lastIntake, intakePower)) { intake.setPower(intakePower); lastIntake = intakePower; }
        if (changed(lastCrLeft, leftPower)) { leftCr.setPower(leftPower); lastCrLeft = leftPower; }
        if (changed(lastCrRight, rightPower)) { rightCr.setPower(rightPower); lastCrRight = rightPower; }
    }

    private void setTransfer(double p) {
        if (changed(lastTransfer, p)) { transfer.setPower(p); lastTransfer = p; }
    }

    private void setGate(double pos) {
        if (changed(lastGate, pos)) { gate.setPosition(pos); lastGate = pos; }
    }

    /** Picks the PIDF tier from battery voltage, with hysteresis; only writes on a tier change. */
    private void applyPidfForVoltage(double v) {
        int tier;
        if (pidfTier == -1) {
            tier = v <= LOW_VOLT ? 0 : (v >= HIGH_VOLT ? 2 : 1);
        } else {
            tier = pidfTier;
            if (pidfTier == 1) {
                if (v <= LOW_VOLT - VOLT_HYST) tier = 0;
                else if (v >= HIGH_VOLT + VOLT_HYST) tier = 2;
            } else if (pidfTier == 0 && v > LOW_VOLT + VOLT_HYST) {
                tier = v >= HIGH_VOLT + VOLT_HYST ? 2 : 1;
            } else if (pidfTier == 2 && v < HIGH_VOLT - VOLT_HYST) {
                tier = v <= LOW_VOLT - VOLT_HYST ? 0 : 1;
            }
        }
        if (tier == pidfTier) return;
        pidfTier = tier;
        PIDFCoefficients c = tier == 0 ? pidfLow : (tier == 2 ? pidfHigh : pidfMid);
        outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
    }
}