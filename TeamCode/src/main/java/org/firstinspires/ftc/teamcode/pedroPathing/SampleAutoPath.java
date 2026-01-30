package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class SampleAutoPath extends OpMode{

    //==========Motor Stuffs=========\\
    DcMotorEx Launcher;
    DcMotorEx Launcher2;
    DcMotorEx indexer;
    Servo forkservo;
    Servo forkservo2;


    void initHardware() {
        //Motors init
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        Launcher2 = hardwareMap.get(DcMotorEx.class, "Launcher2");
        indexer = hardwareMap.get(DcMotorEx.class , "indexer");
        forkservo = hardwareMap.get(Servo.class, "forkservo");
        forkservo2 = hardwareMap.get(Servo.class, "forkservo2");

        //Motors direction
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        Launcher2.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(DcMotor.Direction.REVERSE);

        //Motors run mode
        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Motors zero power
        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //===Flywheel Logic===\\
    void startFlywheel() {
        Launcher.setVelocity(1100);
        Launcher2.setVelocity(1100);
    }

    void stopFlywheel() {
        Launcher.setPower(0);
        Launcher2.setPower(0);
    }

    boolean flywheelAtSpeed() {
        return Math.abs(Launcher.getVelocity() - 1100) < 50
                && Math.abs(Launcher2.getVelocity() - 1100) < 50;
    }


    void moveForkServos() {
        forkservo.setPosition(0.9);
        forkservo2.setPosition(0.6);
    }

    void feedArtifact() {
        indexer.setPower(1);
    }
    //
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE >MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        SERVO_MOVE,
        DRIVE_SHOOTPOS_ENDPOS
        }

    PathState pathState;

    private final Pose startPose = new Pose(21.22077922077922, 121.84415584415584, Math.toRadians(135));
    private final Pose shootPose = new Pose(55.37662337662337, 88.0909090909091, Math.toRadians(135));

    private final Pose endPose = new Pose(46.76521739130433, 113.25217391304346, Math.toRadians(300));

    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:

                startFlywheel();

                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset the timer & make new state
                break;


            case SHOOT_PRELOAD:

                if (!follower.isBusy()
                        && flywheelAtSpeed()
                        && pathTimer.getElapsedTimeSeconds() > 5) {

                    moveForkServos();
                    setPathState(PathState.SERVO_MOVE);
                }
                break;


            case SERVO_MOVE:
                if (pathTimer.getElapsedTimeSeconds() > 0.5
                        && pathTimer.getElapsedTimeSeconds() < 0.8) {

                    feedArtifact();

                } else if (pathTimer.getElapsedTimeSeconds() >= 0.8){

                    indexer.setPower(0);
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                }
                break;


            case  DRIVE_SHOOTPOS_ENDPOS:
                if (!follower.isBusy()) {
                    stopFlywheel();
                    telemetry.addLine("Done Path 2");
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;

        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        initHardware();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        if(pathState != PathState.SERVO_MOVE) {
            indexer.setPower(0);
        }

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}
