package org.firstinspires.ftc.teamcode.Auton; // make sure this aligns with class location

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import static com.pedropathing.ivy.Scheduler.*;
//import static com.pedropathing.ivy.pedro.PedroCommands.*;
//import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.util.Timer;


@Autonomous(name = "Basic Auto")


public class BasicAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, totalTimer;

    //enum is a bunch of constants
    public enum PathState {
        //always gives a start position and end position
        DRIVE_STARTPOS_LOAD_POS,
        LOAD_POS_DRIVE_STARTPOS
    }
    /*
    In general, Pedro pathing works by making a bunch of paths going from position 1 to 2, position 2 to 3, etc
     */


    /*
    Poses are set position for your robot to go. first two arguments are x and y, then it's the angle (in radians, 0.5 is forward.
    use visualizer.pedropathing.com
     */

    PathState pathState;
    private final Pose centerPose = new Pose(72,72, 0.5);
    private final Pose grabPose = new Pose(120, 72, 0.5);



    private PathChain centertograb;
    private PathChain grabtocenter;
    // above is the chain, the code for auton travel, (each path must need its own pathchain)

    public void buildPaths() {
        //enter coordinates for start then end
        centertograb = follower.pathBuilder()
                .addPath(new BezierLine(centerPose, grabPose))  //added line movement(Bezier line)
                .setLinearHeadingInterpolation(centerPose.getHeading(), grabPose.getHeading())
                .build();
        grabtocenter = follower.pathBuilder()
                .addPath(new BezierLine(grabPose, centerPose))  //added line movement(Bezier line)
                .setLinearHeadingInterpolation(grabPose.getHeading(), centerPose.getHeading())
                .build();
    }

    //updating paths
    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_LOAD_POS:
                follower.followPath(centertograb, true); //holdEnd makes the robot hold the position after the path ends
                //pathState = PathState.LOAD_POS_DRIVE_STARTPOS; //switches to the next state
                setPathState(PathState.LOAD_POS_DRIVE_STARTPOS); //reset timer and make new state
                break;
            case LOAD_POS_DRIVE_STARTPOS:
                if(!follower.isBusy()) {
                    follower.followPath(grabtocenter, true);
                    //pathState = PathState.DRIVE_STARTPOS_LOAD_POS;
                    telemetry.addLine("Finished1");
                    break;
                }
            default:
                telemetry.addLine("Nothing to do");
                break;

        }
    }

    public void setPathState(PathState nextState) {
        pathState = nextState;
        pathTimer.resetTimer();
    }
    @Override
    public void init () {
        //inits
        pathState = PathState.DRIVE_STARTPOS_LOAD_POS;
        pathTimer = new Timer();
        totalTimer = new Timer();
        totalTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        // add in everything other inits

        buildPaths();
        follower.setPose(centerPose);
        setPathState(pathState);

    //hi
    }

    public void start() {
        //totalTimer.resetTimer();
    }
    @Override
    public void loop () {
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
    }

}
