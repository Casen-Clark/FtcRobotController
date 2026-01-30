package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous
public class TestAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer , opModeTimer;
    //=====================motor stuff
    DcMotorEx LeftFront, RightFront, LeftRear, RightRear;
    DcMotorEx Launcher, IntakeMotor, indexer;
    CRServo LeftIntake, RightIntake;
    Servo forkservo;

    //================Set Poses===========\\
    //in front of goal directly over launch line with front corner touching goal
    final Pose startPose = new Pose(20.04633204633205, 122.90347490347492, Math.toRadians(137));


    //launches close to the goal and a bit off center to optimise for time
    final Pose LaunchPoseExtraClose = new Pose(32.803088803088784, 103.04247104247098, 129);



    //default launch spot in the close launch zone
    final Pose LaunchPoseClose = new Pose(56.9, 83.95, 136);


    //default Launch spot in the far launch zone
    final Pose LaunchPoseFar = new Pose();


    //lined up backward to begin in taking first spike mark
    final Pose SpikeStart1 = new Pose(44.65250965250966, 83.79536679536677, 0);

    //stop in taking first spike mark
    final Pose SpikeEnd1 = new Pose(15.208494208494205, 83.6911196911197, 0);


    //lined up backward to begin in taking first spike mark
    final Pose SpikeStart2 = new Pose(57.15, 59.4, 0);
    final Pose SpikeEnd2 = new Pose(8.7, 58.699999999999996, 0);
    //=======================================================================\\

    private PathChain start_laucnExtraClose;
    private PathChain launcExtraClose_spike1;
    private PathChain spike1_spike2;

    public void buildPaths(){
        //connects dots(poses) to make lines for the robot to follow
        start_laucnExtraClose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, LaunchPoseExtraClose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())
                .build();
        launcExtraClose_spike1 = follower.pathBuilder()
                .addPath(new BezierLine(LaunchPoseExtraClose, SpikeStart1))
                .setLinearHeadingInterpolation(LaunchPoseExtraClose.getHeading(), SpikeStart1.getHeading())
                .build();

    }

    public enum PathState {
        LAUNCH_SEQUENCE,
        LAUNCH_PRELOADS,
        GT_SPIKE_1,
        IN_SPIKE_1,
        GT_LAUNCH_1,
        GT_SPIKE_2,
        INTAKE_SPIKE_2,
    }
    PathState pathState;
    public void statePathUpdate(){
        switch(pathState){
            case LAUNCH_PRELOADS:
                follower.followPath(start_laucnExtraClose, true);
                transitionPathState(PathState.LAUNCH_SEQUENCE);
                break;
            case GT_SPIKE_1:
                follower.followPath(launcExtraClose_spike1, true);
                break;
            case IN_SPIKE_1:
                break;
            case GT_LAUNCH_1:
                break;
            case GT_SPIKE_2:
                break;
            case INTAKE_SPIKE_2:
                break;
            case LAUNCH_SEQUENCE:
                if(!follower.isBusy()){
                    launch();
                    transitionPathState(PathState.GT_SPIKE_1);
                }
                break;
        }
    }

    public void transitionPathState(PathState newPath){
        pathState = newPath;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.LAUNCH_PRELOADS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);

    }
    public void start(){
        opModeTimer.resetTimer();
        transitionPathState(pathState.LAUNCH_PRELOADS);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
    boolean launch(){
        return false;
    }
    /* ================= LAUNCH =================
    void launchTriple() {
        for (int i=0;i<LAUNCH_CYCLES && opModeIsActive();i++){
            // Lower fork
            forkservo.setPosition(FORK_DOWN);
            sleep(FORK_MOVE_DELAY_MS);

            // Spin indexer at constant power (no RUN_TO_POSITION)
            indexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            indexer.setPower(INDEXER_POWER);

            // Let it spin long enough to push the ball (~time to rotate 120 degrees)
            sleep(1500); // tune this if needed

            // Stop indexer
            indexer.setPower(0);

            // Small wait before fork lifts
            sleep(300);

            // Wait until flywheel reaches speed
            while(getLauncherRPM()<LAUNCH_RPM && opModeIsActive()) idle();

            // Lift fork
            forkservo.setPosition(FORK_UP);
            sleep(FORK_MOVE_DELAY_MS);
        }
    }

    void spinUpLauncher() {
        Launcher.setVelocity(LAUNCH_RPM*LAUNCHER_TICKS_PER_REV/60.0);
    }*/
}
