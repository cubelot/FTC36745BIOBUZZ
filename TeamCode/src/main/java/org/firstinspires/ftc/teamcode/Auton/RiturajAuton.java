package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;




@Autonomous(name = "Comp2AutonUnder")
public class RiturajAuton extends OpMode {

    private Follower follower;

    private DcMotor outtake;
    private DcMotor intake;

    private DcMotor transfer;

    private Servo flowerServo1;
    private Servo flowerServo2;
    private Servo gate;

    private VoltageSensor voltage;






    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose shoottogarden = poseFactory.of(60, 9.1, 270);
    private final Pose shoot1Flower1 = poseFactory.of(10.75, 46.75, 180);
    private final Pose shoot1Flower1Control1 = poseFactory.of(16.75, 16.75, 0);
    private final Pose shoot1Flower1Control2 = poseFactory.of(25.75, 52.75, 0);
    private final Pose flower1shoot2Start = poseFactory.of(10.75, 46.75, 180);
    private final Pose flower1shoot2 = poseFactory.of(60, 132, 90);
    private final Pose flower1shoot2Control1 = poseFactory.of(43.75, 40.75, 0);
    private final Pose flower1shoot2Control2 = poseFactory.of(0, 103.75, 0);
    private final Pose flower1shoot2Control3 = poseFactory.of(55.75, 100.75, 0);
    private final Pose shoot2flower2 = poseFactory.of(46.75, 130.75, 90);
    private final Pose shoot2flower2Control1 = poseFactory.of(54, 96, 0);
    private final Pose flower2shoot2 = poseFactory.of(60, 132, 90);
    private final Pose flower2shoot2Control1 = poseFactory.of(54, 96, 0);
    private final Pose shoot2park = poseFactory.of(12, 120, 90);
    private final Pose shoot2parkControl1 = poseFactory.of(44, 113, 0);



    private final AutonPaths paths = new AutonPaths();


    public class AutonPaths {

        public Path shoot1Flower1() {
            return Paths.curve(shoottogarden, shoot1Flower1Control1, shoot1Flower1Control2, shoot1Flower1).linear(shoottogarden, shoot1Flower1);
        }

        public Path flower1shoot2() {
            return Paths.curve(flower1shoot2Start, flower1shoot2Control1, flower1shoot2Control2, flower1shoot2Control3, flower1shoot2).linear(flower1shoot2Start, flower1shoot2);
        }

        public Path shoot2flower2() {
            return Paths.curve(flower1shoot2, shoot2flower2Control1, shoot2flower2).constant(shoot2flower2);
        }

        public Path flower2shoot2() {
            return Paths.curve(shoot2flower2, flower2shoot2Control1, flower2shoot2).constant(flower2shoot2);
        }

        public Path shoot2park() {
            return Paths.curve(flower2shoot2, shoot2parkControl1, shoot2park).constant(shoot2park);
        }
    }


    @Override
    public void init() {

        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");

        flowerServo1 = hardwareMap.get(Servo.class, "flower1");
        flowerServo2 = hardwareMap.get(Servo.class, "flower2");

        gate = hardwareMap.get(Servo.class, "gate");

        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");


        outtake.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);


        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        follower = Constants.create(hardwareMap);

        follower.setPose(shoottogarden);

        Scheduler.reset();

    }


    @Override
    public void start() {

        schedule(
                sequential(

                        shoot(),

                        follow(follower, paths.shoot1Flower1()),

                        flowerIntake(),

                        follow(follower, paths.flower1shoot2()),

                        shoot(),

                        follow(follower, paths.shoot2flower2()),

                        flowerIntake(),

                        follow(follower, paths.flower2shoot2()),

                        shoot(),

                        follow(follower, paths.shoot2park())

                )
        );

    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("Voltage", voltage.getVoltage());
        telemetry.update();

    }


    @Override
    public void stop() {

        Scheduler.reset();

        if (outtake != null) outtake.setPower(0);
        if (intake != null) intake.setPower(0);
        if (transfer != null) transfer.setPower(0);
        OpModeStorage.autonomousEndPose = follower.pose();
    }

    public Command flowerIntake() {
        return sequential(
                instant(() -> {
                    flowerServo1.setPosition(0.4);
                    flowerServo2.setPosition(0.6);
                    intake.setPower(1);
                }),
                waitMs(5000),
                instant(() -> {
                    intake.setPower(0);
                    flowerServo1.setPosition(1);
                    flowerServo2.setPosition(0);
                })
        );
    }


    public Command shoot() {

        return sequential(

                instant(() -> outtake.setPower(0.69)),

                waitMs(5000),

                instant(() -> {
                    transfer.setPower(0.5);
                    intake.setPower(1);
                    gate.setPosition(0.2);
                }),

                waitMs(3000),

                instant(() -> {
                    outtake.setPower(0);
                    transfer.setPower(0);
                    intake.setPower(0);
                    gate.setPosition(0.5);
                })

        );

    }


}
