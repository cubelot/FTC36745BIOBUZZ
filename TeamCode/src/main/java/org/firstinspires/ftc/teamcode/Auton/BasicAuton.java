//package org.firstinspires.ftc.teamcode.Auton; // make sure this aligns with class location
//import org.firstinspires.ftc.teamcode.Auton.Drawing;
//import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import static java.lang.Math.toRadians;
//
//import com.pedropathing.follower.Follower;
////import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
////import com.pedropathing.ivy.Command;
////import com.pedropathing.ivy.Scheduler;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import static com.pedropathing.ivy.Scheduler.*;
////import static com.pedropathing.ivy.pedro.PedroCommands.*;
////import static com.pedropathing.ivy.groups.Groups.*;
//import com.pedropathing.util.Timer;
//
//
//@Autonomous(name = "Basic Auto")
//
//
//public class BasicAuton extends OpMode {
//
//    private Follower follower;
//    private Timer pathTimer, totalTimer;
//
//    //enum is a bunch of constants
//    public enum PathState {
//        //always gives a start position and end position
//        MOVE_RIGHT,
//        MOVE_UP,
//        MOVE_BACK
//    }
//    /*
//    In general, Pedro pathing works by making a bunch of paths going from position 1 to 2, position 2 to 3, etc
//     */
//
//
//    /*
//    Poses are set position for your robot to go. first two arguments are x and y, then it's the angle (in radians, 0.5 is forward.
//    use visualizer.pedropathing.com
//     */
//
//    PathState pathState;
//    private final Pose centerPose = new Pose(72,72, Math.toRadians(90));
//    private final Pose rightPose = new Pose(120, 72, Math.toRadians(0));
//    private final Pose endPose = new Pose(120,120, Math.toRadians(270));
//
//
//
//    private PathChain moveright;
//    private PathChain moveup;
//    private PathChain moveback;
//    // above is the chain, the code for auton travel, (each path must need its own pathchain)
//
//    public void buildPaths() {
//        //enter coordinates for start then end
//        moveup = follower.pathBuilder()
//                .addPath(new BezierLine(centerPose, rightPose))  //added line movement(Bezier line)
//                .setLinearHeadingInterpolation(centerPose.getHeading(), rightPose.getHeading())
//                .build();
//        moveright = follower.pathBuilder()
//                .addPath(new BezierLine(rightPose, endPose))  //added line movement(Bezier line)
//                .setLinearHeadingInterpolation(rightPose.getHeading(), endPose.getHeading())
//                .build();
//        moveback = follower.pathBuilder()
//                .addPath(new BezierLine(endPose, centerPose))
//                .setLinearHeadingInterpolation(endPose.getHeading(), centerPose.getHeading())
//                .build();
//    }
//
//    //updating paths
//    public void statePathUpdate() {
//        switch(pathState) {
//            case MOVE_RIGHT:
//                follower.followPath(moveright, true);//holdEnd makes the robot hold the position after the path ends
//                //pathState = PathState.LOAD_POS_DRIVE_STARTPOS; //switches to the next state
//                setPathState(PathState.MOVE_UP); //reset timer and make new state
//                break;
//            case MOVE_UP:
//                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
//                    follower.followPath(moveup, true);//holdEnd makes the robot hold the position after the path ends
//                    //pathState = PathState.LOAD_POS_DRIVE_STARTPOS; //switches to the next state
//                    setPathState(PathState.MOVE_BACK); //reset timer and make new state
//                    break;
//                }
//            case MOVE_BACK:
//                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
//                    follower.followPath(moveback, true);
//                    //pathState = PathState.DRIVE_STARTPOS_LOAD_POS;
//                    telemetry.addLine("Finished1");
//                    break;
//                }
//            default:
//                telemetry.addLine("Nothing to do");
//                break;
//
//        }
//    }
//
//    public void setPathState(PathState nextState) {
//        pathState = nextState;
//        pathTimer.resetTimer();
//    }
//    @Override
//    public void init () {
//        //inits
//        pathState = PathState.MOVE_RIGHT;
//        pathTimer = new Timer();
//        totalTimer = new Timer();
//        totalTimer.resetTimer();
//        follower = Constants.createFollower(x);
//        // add in everything other inits
//
//        buildPaths();
//        follower.setPose(centerPose);
//        setPathState(pathState);
//
//    //hi
//    }
//
//    public void start() {
//        //totalTimer.resetTimer();
//    }
//    @Override
//    public void loop () {
//        follower.update();
//
//        Drawing.drawDebug(follower);
//        statePathUpdate();
//        telemetry.addData("path state", pathState.toString());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.update();
//    }
//
//}
