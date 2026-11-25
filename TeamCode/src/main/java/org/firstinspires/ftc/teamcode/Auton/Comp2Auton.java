package org.firstinspires.ftc.teamcode.Auton;
import static com.pedropathing.api.Paths.*;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;






@Autonomous(name = "Comp 2 Auton")
//built by anay
public class Comp2Auton extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final ElapsedTime shootTime = new ElapsedTime();
    private DcMotor outtake;
    private DcMotor intake;
    private DcMotor transfer;



    private Servo flowerServo1;
    private Servo flowerServo2;

    private Servo gate;


    //cute
    private VoltageSensor voltage;










    private boolean isShootDone = false;
    private boolean isIntakeDone = false;
    private AutonPaths paths = new AutonPaths();




    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose shoot1 = poseFactory.of(60, 12, 270);
    private final Pose garden = poseFactory.of(9, 10, 270);
    private final Pose shoot2 = poseFactory.of(60, 132, 270);
    private final Pose flower = poseFactory.of(48, 126, 270);
    private final Pose park   = poseFactory.of(10, 95, 270);

    private final Pose gardenCtrl  = poseFactory.of(32, 24, 0);
    private final Pose shoot2Ctrl1 = poseFactory.of(56, 38, 0);
    private final Pose shoot2Ctrl2 = poseFactory.of(0, 88, 0);
    private final Pose shoot2Ctrl3 = poseFactory.of(26, 111, 0);
    private final Pose shoot2Ctrl4 = poseFactory.of(66, 112, 0);
    private final Pose flowerCtrl  = poseFactory.of(54, 116, 0);
    private final Pose flowerBackCtrl = poseFactory.of(54, 116, 0);
    private final Pose parkCtrl    = poseFactory.of(83, 114, 0);

    public class AutonPaths {
        public Path shoot1ToGarden() {
            return curve(shoot1, gardenCtrl, garden).linear(shoot1, garden);
        }

        public Path gardenToShoot2() {
            return curve(garden, shoot2Ctrl1, shoot2Ctrl2, shoot2Ctrl3, shoot2Ctrl4, shoot2).linear(garden, shoot2);
        }

        public Path shoot2flower() {
            return curve(shoot2, flowerCtrl, flower).linear(shoot2, flower);
        }

        public Path flowerToShoot2() {
            return curve(flower, flowerBackCtrl, shoot2).linear(flower, shoot2);
        }

        public Path shoot2park() {
            return curve(shoot2, parkCtrl, park).linear(shoot2, park);
        }
    }





    @Override
    public void init() {




        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");


        flowerServo1 = hardwareMap.get(Servo.class, "flower1");
        flowerServo2 = hardwareMap.get(Servo.class, "flower2");


        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

        gate = hardwareMap.get(Servo.class, "gate");


        outtake.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);








        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        PIDFCoefficients pidfCoefficients1 =
                new PIDFCoefficients(44.5, 0, 0, 14.9469);


        PIDFCoefficients pidfCoefficients2 =
                new PIDFCoefficients(0, 0, 0, 15.5469);


        PIDFCoefficients pidfCoefficients3 =
                new PIDFCoefficients(25.3, 0, 0, 14.65);


        /*if (voltage.getVoltage() <= 12.25) {
            outtake.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    pidfCoefficients2
            );
        } else if (voltage.getVoltage() >= 13) {
            outtake.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    pidfCoefficients3
            );
        } else {
            outtake.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    pidfCoefficients1
            );
        }*/


        // Initialize scheduler
        Scheduler.reset();


        // Initialize follower
        follower = Constants.create(hardwareMap);
        follower.setPose(shoot1);
    }




    @Override
    public void start() {
        schedule(
                sequential(
                        shoot(),
                        follow(follower, paths.shoot1ToGarden()),
                        intakeGarden(),
                        follow(follower, paths.gardenToShoot2()),
                        shoot(),
                        follow(follower, paths.shoot2flower()),
                        flowerIntake(),
                        follow(follower, paths.flowerToShoot2()),
                        follow(follower, paths.shoot2park())
                )
        );


    }




    @Override
    public void loop() {


        follower.update();


        telemetry.addData("Voltage", voltage.getVoltage());



        telemetry.update();


        Scheduler.execute();
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

                    isIntakeDone = true;
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
                    isShootDone = true;
                } )
        );


    }


    public Command flowerIntake() {
        return sequential(
                instant(() -> flowerServo1.setPosition(0.4)),
                instant(() -> flowerServo2.setPosition(0.6)),
                instant(() -> intake.setPower(1)),
                waitMs(5000),
                instant(() -> flowerServo1.setPosition(1)),
                instant(() -> flowerServo2.setPosition(0))
        );
    }


}



