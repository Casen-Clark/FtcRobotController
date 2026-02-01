package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//MOTOR SETUP - 24
//LAUNCHER LOGIC - 72
//INTAKE LOGIC - 165
//PATHING STUFF - 185

@Autonomous
public class BlueAuto extends OpMode{

    //==========Motor Stuffs=========\\
    DcMotorEx Launcher;
    DcMotorEx Launcher2;
    DcMotorEx indexer;
    DcMotorEx IntakeMotor;
    Servo forkservo;
    Servo forkservo2;
    CRServo LeftIntake;
    CRServo RightIntake;
    CRServo LeftBandintake;
    CRServo RightBandintake;


    void initHardware() {
        //Motors init
        Launcher = hardwareMap.get(DcMotorEx.class, "Launcher");
        Launcher2 = hardwareMap.get(DcMotorEx.class, "Launcher2");
        indexer = hardwareMap.get(DcMotorEx.class , "indexer");
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        forkservo = hardwareMap.get(Servo.class, "forkservo");
        forkservo2 = hardwareMap.get(Servo.class, "forkservo2");
        LeftIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        RightIntake = hardwareMap.get(CRServo.class, "RightIntake");
        LeftBandintake = hardwareMap.get(CRServo.class, "LeftBandintake");
        RightBandintake = hardwareMap.get(CRServo.class, "RightBandintake");

        //Motors direction
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        Launcher2.setDirection(DcMotor.Direction.REVERSE);
        indexer.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftIntake.setDirection(CRServo.Direction.FORWARD);
        RightIntake.setDirection(CRServo.Direction.REVERSE);
        LeftBandintake.setDirection(CRServo.Direction.REVERSE);
        RightBandintake.setDirection(CRServo.Direction.FORWARD);

        //Motors run mode
        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Motors zero power
        Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                Launcher.setVelocityPIDFCoefficients(5, 2, 3, 4);
                Launcher2.setVelocityPIDFCoefficients(5, 2, 3, 4);

    }

    //-----------------Launcher Logic-------------------------\\
    LauncherState launcherState = LauncherState.IDLE;
    Timer launcherTimer = new Timer();

    void LaunchArtifacts(Pose targetPose) {
        // Start launcher if robot is within tolerance
        if (launcherState == LauncherState.IDLE) {
            Pose current = follower.getPose();
            double dx = Math.abs(current.getX() - targetPose.getX());
            double dy = Math.abs(current.getY() - targetPose.getY());
            double dHeading = Math.abs(current.getHeading() - targetPose.getHeading());

            if (dx < POSITION_TOLERANCE && dy < POSITION_TOLERANCE && dHeading < HEADING_TOLERANCE) {
                launcherState = LauncherState.WAIT_FOR_FLYWHEEL;
                launcherTimer.resetTimer();
            }
        }
    }
    // Launch tolerance
    double POSITION_TOLERANCE = 2.0; // +/- in inches
    double HEADING_TOLERANCE = Math.toRadians(3); // +/- in degrees

    void updateLauncher() {
        switch (launcherState) {

            case IDLE:
                break;

            case WAIT_FOR_FLYWHEEL:
                startFlywheel();
                if (flywheelAtSpeed()) {
                    launcherState = LauncherState.OPEN_GATE;
                    launcherTimer.resetTimer();
                }
                break;

            case OPEN_GATE:
                forkservo.setPosition(0.9);
                forkservo2.setPosition(0.6);
                if (launcherTimer.getElapsedTimeSeconds() > 0.5) {
                    launcherState = LauncherState.FEEDING;
                    launcherTimer.resetTimer();
                }
                break;

            case FEEDING:
                indexer.setPower(1);
                if (launcherTimer.getElapsedTimeSeconds() > 2.0) {
                    indexer.setPower(0);
                    launcherState = LauncherState.CLOSING;
                }
                break;

            case CLOSING:
                forkservo.setPosition(0.65);
                forkservo2.setPosition(0.85);
                launcherState = LauncherState.DONE;
                break;

            case DONE:
                stopFlywheel();
                launcherState = launcherState.IDLE;
                break;
        }
    }

    void resetLauncher() {
        launcherState = LauncherState.IDLE;
    }

    void startFlywheel() {
        Launcher.setVelocity(1000);
        Launcher2.setVelocity(1000);
    }

    void stopFlywheel() {
        Launcher.setPower(0);
        Launcher2.setPower(0);
    }

    boolean flywheelAtSpeed() {
        return Math.abs(Launcher.getVelocity() - 1000) < 50
                && Math.abs(Launcher2.getVelocity() - 1000) < 50;
    }

    boolean launchComplete() {
        return launcherState == LauncherState.DONE;
    }

    public enum LauncherState {
        IDLE,
        WAIT_FOR_FLYWHEEL,
        OPEN_GATE,
        FEEDING,
        CLOSING,
        DONE
    }

    //----------------Intake Logic----------------------------\\

    public void startIntake() {
        IntakeMotor.setVelocity(600);
        LeftIntake.setPower(1);
        RightIntake.setPower(1);
    }

    void updateIntake() {
        /*
        ,
        LAUNCHPOSE_STARTPICKUPSPIKE1,
        STARTPICKUPSPIKE1_ENDPICKUPSPIKE1,
        ENDPICKUPSPIKE1_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPSPIKE2,
        STARTPICKUPSPIKE2_ENDPICKUPSPIKE2,
        ENDPICKUPSPIKE2_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPSPIKE3,
        STARTPICKUPSPIKE3_ENDPICKUPSPIKE3,
        E
        */
        boolean intakeActive = pathState == PathState.STARTPOSE_LAUNCHPOSE || pathState == PathState.ENDPICKUPSPIKE3_ENDPOSE;

        if (!intakeActive) {
            // Turn on intake motors
            IntakeMotor.setVelocity(900);
            indexer.setPower(1);
            LeftIntake.setPower(1);
            RightIntake.setPower(1);
            LeftBandintake.setPower(1);
            RightBandintake.setPower(1);
        } else {
            // Stop intake motors
            IntakeMotor.setPower(0);

            LeftIntake.setPower(0);
            RightIntake.setPower(0);
            LeftBandintake.setPower(0);
            RightBandintake.setPower(0);

        }
    }


    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //=======================================PATHING STUFF========================================\\
    public enum PathState {
        STARTPOSE_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPSPIKE1,
        STARTPICKUPSPIKE1_ENDPICKUPSPIKE1,
        ENDPICKUPSPIKE1_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPSPIKE2,
        STARTPICKUPSPIKE2_ENDPICKUPSPIKE2,
        ENDPICKUPSPIKE2_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPSPIKE3,
        STARTPICKUPSPIKE3_ENDPICKUPSPIKE3,
        ENDPICKUPSPIKE3_ENDPOSE
    }

    PathState pathState;

    private final Pose startPose = new Pose(21.22077922077922, 121.84415584415584, Math.toRadians(135));//START POSE
    private final Pose launchPose = new Pose(55.37662337662337, 88.0909090909091, Math.toRadians(135));
    private final Pose startPickupSpike1 = new Pose(45.97258687258688, 82.68725868, Math.toRadians(0));//MOVE TO PICKUP 1ST SPIKE
    private final Pose endPickupSpike1 = new Pose(17.470656370656375, 82.68725868, Math.toRadians(0));//PICKUP 1ST SPIKE
    private final Pose startPickupSpike2 = new Pose(42.06956521739129, 58.06956521739131, Math.toRadians(0));//MOVE TO PICKUP 2ND SPIKE
    private final Pose endPickupSpike2 = new Pose(17.6868725868725965, 57.16521739130434, Math.toRadians(0));//PICKUP 2ND SPIKE
    private final Pose startPickupSpike3 = new Pose(40.66086956521737, 34.8782608695652, Math.toRadians(0));//MOVE TO PICKUP 2ND SPIKE
    private final Pose endPickupSpike3 = new Pose(17.686872586872596, 34.42857142857139, Math.toRadians(0));//PICKUP 2ND SPIKE
    private final Pose endPose = new Pose(30.99961389961389, 71.56756756756754, Math.toRadians(90));//LAUNCH FINAL 3 ARTIFACTS + LEAVE

    private PathChain
            startPose_LaunchPose,
            launchPose_StartPickupSpike1,
            startPickupSpike1_EndPickupSpike1,
            endPickupSpike1_launchPose,
            launchPose_StartPickupSpike2,
            startPickupSpike2_EndPickupSpike2,
            endPickupSpike2_LaunchPose,
            launchPose_StartPickupSPike3,
            startPickupSpike3_EndPickupSPike3,
            endPickupSpike3_endPose;

    public void buildPaths() {

        // Move from start to launch pose
        startPose_LaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();

        // Move to pickup first spike mark
        launchPose_StartPickupSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickupSpike1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickupSpike1.getHeading())
                .build();

        // Pickup first spike mark
        startPickupSpike1_EndPickupSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(startPickupSpike1, endPickupSpike1))
                .setLinearHeadingInterpolation(startPickupSpike1.getHeading(), endPickupSpike1.getHeading())
                .build();

        // Move back to launch pose
        endPickupSpike1_launchPose = follower.pathBuilder()
                .addPath(new BezierLine(endPickupSpike1, launchPose))
                .setLinearHeadingInterpolation(endPickupSpike1.getHeading(), launchPose.getHeading())
                .build();

        // Move to pickup second spike mark
        launchPose_StartPickupSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickupSpike2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickupSpike2.getHeading())
                .build();

        // Pickup second spike mark
        startPickupSpike2_EndPickupSpike2 = follower.pathBuilder()
                .addPath(new BezierLine(startPickupSpike2, endPickupSpike2))
                .setLinearHeadingInterpolation(startPickupSpike2.getHeading(), endPickupSpike2.getHeading())
                .build();

        // Move back to launch pose
        endPickupSpike2_LaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(endPickupSpike2, launchPose))
                .setLinearHeadingInterpolation(endPickupSpike2.getHeading(), launchPose.getHeading())
                .build();

        // Move to pickup third spike mark
        launchPose_StartPickupSPike3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickupSpike3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickupSpike3.getHeading())
                .build();

        // Pickup third spike mark
        startPickupSpike3_EndPickupSPike3 = follower.pathBuilder()
                .addPath(new BezierLine(startPickupSpike3, endPickupSpike3))
                .setLinearHeadingInterpolation(startPickupSpike3.getHeading(), endPickupSpike3.getHeading())
                .build();

        // Move to final launch and end pose
        endPickupSpike3_endPose = follower.pathBuilder()
                .addPath(new BezierLine(endPickupSpike3, endPose))
                .setLinearHeadingInterpolation(endPickupSpike3.getHeading(), endPose.getHeading())
                .build();
    }

    boolean pathStarted = false;
    public void statePathUpdate() {
        switch (pathState) {

            case STARTPOSE_LAUNCHPOSE:
                // Start path if not already started
                if (!pathStarted) {
                    follower.followPath(startPose_LaunchPose, true);
                    pathStarted = true;
                }

                // Launch automatically when at launchPose
                LaunchArtifacts(launchPose);

                // Move to next path only when both path and launcher complete
                if (!follower.isBusy() && launchComplete()) {
                    setPathState(PathState.LAUNCHPOSE_STARTPICKUPSPIKE1);
                    pathStarted = false;
                }
                break;

            case LAUNCHPOSE_STARTPICKUPSPIKE1:
                if (!pathStarted) {
                    follower.followPath(launchPose_StartPickupSpike1, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.STARTPICKUPSPIKE1_ENDPICKUPSPIKE1);
                    pathStarted = false;
                }
                break;

            case STARTPICKUPSPIKE1_ENDPICKUPSPIKE1:
                follower.setMaxPowerScaling(0.35);
                if (!pathStarted) {

                    follower.followPath(startPickupSpike1_EndPickupSpike1, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.ENDPICKUPSPIKE1_LAUNCHPOSE);
                    pathStarted = false;
                }
                break;

            case ENDPICKUPSPIKE1_LAUNCHPOSE:
                follower.setMaxPowerScaling(1);
                if (!pathStarted) {

                    follower.followPath(endPickupSpike1_launchPose, true);
                    pathStarted = true;
                }

                // Automatically launch when at launchPose
                LaunchArtifacts(launchPose);

                if (!follower.isBusy() && launchComplete()) {
                    setPathState(PathState.LAUNCHPOSE_STARTPICKUPSPIKE2);
                    pathStarted = false;
                }
                break;

            case LAUNCHPOSE_STARTPICKUPSPIKE2:
                if (!pathStarted) {
                    follower.followPath(launchPose_StartPickupSpike2, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.STARTPICKUPSPIKE2_ENDPICKUPSPIKE2);
                    pathStarted = false;
                }
                break;

            case STARTPICKUPSPIKE2_ENDPICKUPSPIKE2:
                follower.setMaxPowerScaling(0.35);
                if (!pathStarted) {

                    follower.followPath(startPickupSpike2_EndPickupSpike2, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.ENDPICKUPSPIKE2_LAUNCHPOSE);
                    pathStarted = false;
                }
                break;

            case ENDPICKUPSPIKE2_LAUNCHPOSE:
                follower.setMaxPowerScaling(1);
                if (!pathStarted) {

                    follower.followPath(endPickupSpike2_LaunchPose, true);
                    pathStarted = true;
                }

                // Launch automatically when at launchPose
                LaunchArtifacts(launchPose);

                if (!follower.isBusy() && launchComplete()) {
                    setPathState(PathState.LAUNCHPOSE_STARTPICKUPSPIKE3);
                    pathStarted = false;
                }
                break;

            case LAUNCHPOSE_STARTPICKUPSPIKE3:
                if (!pathStarted) {
                    follower.followPath(launchPose_StartPickupSPike3, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.STARTPICKUPSPIKE3_ENDPICKUPSPIKE3);
                    pathStarted = false;
                }
                break;

            case STARTPICKUPSPIKE3_ENDPICKUPSPIKE3:
                follower.setMaxPowerScaling(0.35);
                if (!pathStarted) {

                    follower.followPath(startPickupSpike3_EndPickupSPike3, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.ENDPICKUPSPIKE3_ENDPOSE);
                    pathStarted = false;
                }
                break;

            case ENDPICKUPSPIKE3_ENDPOSE:
                follower.setMaxPowerScaling(1);
                indexer.setPower(0);
                if (!pathStarted) {

                    follower.followPath(endPickupSpike3_endPose, true);
                    pathStarted = true;
                }

                // Launch automatically when at launchPose (if needed)
                //LaunchArtifacts(endPose);

                if (!follower.isBusy() && launchComplete()) {
                    pathStarted = false; // autonomous complete
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

        pathState = PathState.STARTPOSE_LAUNCHPOSE;
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
        updateLauncher();
        statePathUpdate();
        updateIntake();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Launcher State", launcherState);
        telemetry.addData("heading error: ", follower.getHeadingError());
        telemetry.addData("drive error: ", follower.getDriveError());
    }
}
