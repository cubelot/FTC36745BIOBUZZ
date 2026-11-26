
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;







@Autonomous(name = "Comp 2 AutonRP")
public class Comp2AutonRP extends OpMode {

    private Follower follower;

    private DcMotor outtake;
    private DcMotor intake;

    private DcMotor transfer;

    private Servo flowerServo1;
    private Servo flowerServo2;
    private Servo gate;

    private VoltageSensor voltage;









    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Starting and destination poses
    private final Pose shoot1 = poseFactory.of(60, 9.1, 270);

    private final Pose point1 = poseFactory.of(10.75, 46.75, 180);

    private final Pose point1Control1 =
            poseFactory.of(16.75, 16.75, 0);

    private final Pose point1Control2 =
            poseFactory.of(25.75, 52.75, 0);

    private final Pose shoot2Pose =
            poseFactory.of(60, 132, 90);

    private final Pose garden2shoot2Control1 =
            poseFactory.of(43.75, 40.75, 0);

    private final Pose garden2shoot2Control2 =
            poseFactory.of(0, 103.75, 0);

    private final Pose garden2shoot2Control3 =
            poseFactory.of(55.75, 100.75, 0);

    private final Pose flower1Pose =
            poseFactory.of(46.75, 130.75, 90);

    private final Pose shoot2flower1Control1 =
            poseFactory.of(54, 96, 0);

    private final Pose flower1shoot2Control1 =
            poseFactory.of(54, 96, 0);

    private final Pose parkPose =
            poseFactory.of(12, 120, 90);

    private final Pose parkControl1 =
            poseFactory.of(44, 113, 0);


    // Autonomous paths
    private final AutonPaths paths = new AutonPaths();


    public class AutonPaths {

        public Path path1() {
            return Paths.curve(shoot1, point1Control1, point1Control2, point1).linear(shoot1, point1);
        }

        public Path garden2shoot2() {
            return Paths.curve(point1, garden2shoot2Control1, garden2shoot2Control2, garden2shoot2Control3, shoot2Pose).linear(point1, shoot2Pose);
        }

        public Path shoot2flower1() {
            return Paths.curve(shoot2Pose, shoot2flower1Control1, flower1Pose).constant(flower1Pose);
        }

        public Path flower1shoot2() {
            return Paths.curve(flower1Pose, flower1shoot2Control1, shoot2Pose).constant(shoot2Pose);
        }

        public Path path5() {
            return Paths.curve(shoot2Pose, parkControl1, parkPose).constant(parkPose);
        }
    }


    @Override
    public void init() {

        // Hardware initialization
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

        follower.setPose(shoot1);

        Scheduler.reset();

    }


    @Override
    public void start() {

        schedule(
                sequential(

                        shoot(),

                        follow(follower, paths.path1()),

                        intakeGarden(),

                        follow(follower, paths.garden2shoot2()),

                        shoot(),

                        follow(follower, paths.shoot2flower1()),

                        flowerIntake(),

                        follow(follower, paths.flower1shoot2()),

                        follow(follower, paths.path5())

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

        }

    public Command intakeGarden() {

        return sequential(

                instant(() -> {
                    intake.setPower(1);
                }),

                waitMs(5000),

                instant(() -> {
                    intake.setPower(-1);
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



    public Command flowerIntake() {

        return sequential(

                instant(() -> flowerServo1.setPosition(0.4)),

                instant(() -> flowerServo2.setPosition(0.6)),

                instant(() -> intake.setPower(1)),

                waitMs(5000),

                instant(() -> {
                    intake.setPower(0);
                    flowerServo1.setPosition(1);
                    flowerServo2.setPosition(0);
                })

        );

    }

}
