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




    private DcMotorEx outtake;
    private DcMotor intake;
    private DcMotor transfer;




//    private CRServo left;
//    private CRServo right;


    private Servo flowerServo;


    //cute
    private VoltageSensor voltage;










    private boolean isShoot1Done = false;
    private boolean isIntakeDone = false;
    private boolean isShoot2Done = false;
    private AutonPaths paths = new AutonPaths();




    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose shoot1 = poseFactory.of(60, 11.9758, 90);
    private final Pose shoot1ToGarden = poseFactory.of(9.3456, 11.0818, 270);
    private final Pose shoot1ToGardenControl1 = poseFactory.of(31.9897, 25.6839, 0);
    private final Pose gardenToShoot2Start = poseFactory.of(9.3456, 11.0818, 270);
    private final Pose gardenToShoot2 = poseFactory.of(60.42, 131.9784, 270);
    private final Pose gardenToShoot2Control1 = poseFactory.of(56.4328, 38.5049, 0);
    private final Pose gardenToShoot2Control2 = poseFactory.of(0, 87.8898, 0);
    private final Pose gardenToShoot2Control3 = poseFactory.of(25.4666, 110.9095, 0);
    private final Pose gardenToShoot2Control4 = poseFactory.of(65.6565, 111.6322, 0);
    private final Pose shoot2flowerStart = poseFactory.of(60.42, 131.9784, 270);
    private final Pose shoot2flower = poseFactory.of(47.9982, 126.4339, 90);
    private final Pose shoot2flowerControl1 = poseFactory.of(54.4373, 116.1268, 0);
    private final Pose flowerToShoot2Start = poseFactory.of(47.9982, 126.4339, 90);
    private final Pose flowerToShoot2 = poseFactory.of(60.0345, 131.6852, 270);
    private final Pose flowerToShoot2Control1 = poseFactory.of(54.6609, 115.7468, 0);
    private final Pose shoot2parkStart = poseFactory.of(60.0345, 131.6852, 270);
    private final Pose shoot2park = poseFactory.of(9.9416, 94.8113, 270);
    private final Pose shoot2parkControl1 = poseFactory.of(82.7055, 113.9435, 0);
    public class AutonPaths {


        private final PoseFactory p = PoseFactory.degrees();
        private Follower follower;








        public Path shoot1ToGarden() {
            return curve(shoot1, shoot1ToGardenControl1, shoot1ToGarden).linear(shoot1, shoot1ToGarden);
        }


        public Path gardenToShoot2() {
            return curve(gardenToShoot2Start, gardenToShoot2Control1, gardenToShoot2Control2, gardenToShoot2Control3, gardenToShoot2Control4, gardenToShoot2).linear(gardenToShoot2Start, gardenToShoot2);
        }


        public Path shoot2flower() {
            return curve(shoot2flowerStart, shoot2flowerControl1, shoot2flower).linear(shoot2flowerStart, shoot2flower);
        }


        public Path flowerToShoot2() {
            return curve(flowerToShoot2Start, flowerToShoot2Control1, flowerToShoot2).linear(flowerToShoot2Start, flowerToShoot2);
        }


        public Path shoot2park() {
            return curve(shoot2parkStart, shoot2parkControl1, shoot2park).linear(shoot2parkStart, shoot2park);
        }
    }






    @Override
    public void init() {




        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        transfer = hardwareMap.get(DcMotor.class, "Transfer");


        flowerServo = hardwareMap.get(Servo.class, "flowerServo");


        voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");


        outtake.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);




        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        PIDFCoefficients pidfCoefficients1 =
                new PIDFCoefficients(44.5, 0, 0, 14.9469);


        PIDFCoefficients pidfCoefficients2 =
                new PIDFCoefficients(0, 0, 0, 15.5469);


        PIDFCoefficients pidfCoefficients3 =
                new PIDFCoefficients(25.3, 0, 0, 14.65);


        if (voltage.getVoltage() <= 12.25) {
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
        }


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
                        shoot1(),
                        follow(follower, paths.shoot1ToGarden()),
                        intakeGarden(),
                        follow(follower, paths.gardenToShoot2()),
                        shoot2(),
                        follow(follower, paths.shoot2flower()),
                        flowerIntake(),
                        follow(follower, paths.flowerToShoot2()),
                        shoot2(),
                        follow(follower, paths.shoot2park())
                )
        );


    }




    @Override
    public void loop() {


        follower.update();


        telemetry.addData("Voltage", voltage.getVoltage());
        telemetry.addData("Outtake Velocity", outtake.getVelocity());


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


    public Command shoot1() {
        return sequential(
                instant(() -> outtake.setVelocity(1420)),
                waitMs(5000),
                instant(() -> transfer.setPower(0.6)),
                waitMs(3000),
                instant(() -> {
                    outtake.setVelocity(0);
                    transfer.setPower(0);
                    isShoot1Done = true;
                    intake.setPower(-1);
                } )
        );


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


    public Command shoot2() {
        return sequential(
                instant(() -> outtake.setVelocity(1420)),
                waitMs(5000),
                instant(() -> transfer.setPower(0.6)),
                waitMs(3000),
                instant(() -> {
                    outtake.setVelocity(0);
                    transfer.setPower(0);
                    isShoot2Done = true;
                } )
        );


    }


    public Command flowerIntake() {
        return sequential(
                instant(() -> flowerServo.setPosition(0.5)),
                instant(() -> intake.setPower(1)),
                waitMs(5000),
                instant(() -> flowerServo.setPosition(0))
        );
    }


}



