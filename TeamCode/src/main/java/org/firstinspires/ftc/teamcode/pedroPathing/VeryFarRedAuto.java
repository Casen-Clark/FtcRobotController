package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
public class VeryFarRedAuto extends OpMode {

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
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
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
        //indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Launcher.setVelocityPIDFCoefficients(160, 0, 0, 18);
        Launcher2.setVelocityPIDFCoefficients(160, 0, 0, 18); //p100 f12.6

    }

    //-----------------Launcher Logic-------------------------\\
    LauncherState launcherState = LauncherState.IDLE;
    Timer launcherTimer = new Timer();

    void LaunchArtifacts(Pose targetPose) {
        // Start launcher if robot is within tolerance
        telemetry.addLine("thinking about launching");
        if (launcherState == LauncherState.IDLE) {
            Pose current = follower.getPose();
            double dx = Math.abs(current.getX() - targetPose.getX());
            double dy = Math.abs(current.getY() - targetPose.getY());
            double dHeading = Math.abs(current.getHeading() - targetPose.getHeading());

            if (dx < POSITION_TOLERANCE && dy < POSITION_TOLERANCE && dHeading < HEADING_TOLERANCE) {
                telemetry.addLine("Im launching now I decided");
                launcherState = LauncherState.WAIT_FOR_FLYWHEEL;
                launcherTimer.resetTimer();
            }
        }
    }

    // Launch tolerance
    double POSITION_TOLERANCE = 2.0; // +/- in inches //2'
    double HEADING_TOLERANCE = Math.toRadians(5); // +/- in degrees //5

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
                if (launcherTimer.getElapsedTimeSeconds() > 1.0) {
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
        Launcher.setVelocity(1550);
        Launcher2.setVelocity(1550);
    }

    void stopFlywheel() {
        Launcher.setPower(0);
        Launcher2.setPower(0);
    }

    boolean flywheelAtSpeed() {
        return Math.abs(Launcher.getVelocity() - 1190) < 50
                && Math.abs(Launcher2.getVelocity() - 1190) < 50;
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

    boolean isIntakingState() {
        return pathState == PathState.LAUNCHPOSE_STARTPICKUPSPIKE1 ||
                pathState == PathState.STARTPICKUPSPIKE1_ENDPICKUPSPIKE1 ||
                pathState == PathState.STARTPICKUPDEPOT_ENDPICKUPDEPOT;
    }

    public void startIntake() {
        IntakeMotor.setPower(1);
        LeftIntake.setPower(1);
        RightIntake.setPower(1);
    }

    void updateIntake() {

        if (isIntakingState()) {
            IntakeMotor.setPower(1);
            LeftIntake.setPower(1);
            RightIntake.setPower(1);
            LeftBandintake.setPower(1);
            RightBandintake.setPower(1);
        } else {
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
    //=======================================PATHING STUFF========================================\\
    public enum PathState {
        STARTPOSE_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPSPIKE1,
        STARTPICKUPSPIKE1_ENDPICKUPSPIKE1,
        ENDPICKUPSPIKE1_LAUNCHPOSE,
        LAUNCHPOSE_STARTPICKUPDEPOT,
        STARTPICKUPDEPOT_ENDPICKUPDEPOT,
        ENDPICKUPDEPOT_LAUNCHPOSE,
        LAUNCHPOSE_LEAVE,


    }


    PathState pathState;
    //[, , , , , , , ]
    private final Pose startPose = new Pose(96.525, 8.185, Math.toRadians(90));


    private final Pose startPickupSpike1 = new Pose(96.710, 35.259, Math.toRadians(180));
    private final Pose endPickupSpike1   = new Pose(124.347, 35.251, Math.toRadians(180));


    private final Pose launchPose = new Pose(87.429, 13.371, Math.toRadians(67));


    private final Pose startPickupDepot = new Pose(134.888, 28.432, Math.toRadians(95));
    private final Pose endPickupDepot   = new Pose(131.560, 9.151, Math.toRadians(95));


    private final Pose leave = new Pose(111.062, 11.158, Math.toRadians(68));






    private PathChain
            startPose_LaunchPose,
            launchPose_StartPickupSpike1,
            startPickupSpike1_EndPickupSpike1,
            endPickupSpike1_launchPose,
            launchPose_StartPickupDepot,
            startPickupDepot_EndPickupDepot,
            endPickupDepot_LaunchPose,
            launchPose_Leave;


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
        launchPose_StartPickupDepot = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, startPickupDepot))
                .setLinearHeadingInterpolation(launchPose.getHeading(), startPickupDepot.getHeading())
                .build();


        // Pickup second spike mark
        startPickupDepot_EndPickupDepot = follower.pathBuilder()
                .addPath(new BezierLine(startPickupDepot, endPickupDepot))
                .setLinearHeadingInterpolation(startPickupDepot.getHeading(), endPickupDepot.getHeading())
                .build();


        // Move back to launch pose
        endPickupDepot_LaunchPose = follower.pathBuilder()
                .addPath(new BezierLine(endPickupDepot, launchPose))
                .setLinearHeadingInterpolation(endPickupDepot.getHeading(), launchPose.getHeading())
                .build();


        // Move to pickup third spike mark
        launchPose_Leave = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, leave))
                .setLinearHeadingInterpolation(launchPose.getHeading(), leave.getHeading())
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
                    startIntake();
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
                    setPathState(PathState.LAUNCHPOSE_STARTPICKUPDEPOT);
                    pathStarted = false;
                }
                break;


            case LAUNCHPOSE_STARTPICKUPDEPOT:
                if (!pathStarted) {
                    follower.followPath(launchPose_StartPickupDepot, true);
                    pathStarted = true;
                }


                if (!follower.isBusy()) {
                    setPathState(PathState.STARTPICKUPDEPOT_ENDPICKUPDEPOT);
                    pathStarted = false;
                }
                break;


            case STARTPICKUPDEPOT_ENDPICKUPDEPOT:
                follower.setMaxPowerScaling(0.35);
                if (!pathStarted) {


                    follower.followPath(startPickupDepot_EndPickupDepot, true);
                    pathStarted = true;
                }


                if (!follower.isBusy()) {
                    setPathState(PathState.ENDPICKUPDEPOT_LAUNCHPOSE);
                    pathStarted = false;
                }
                break;


            case ENDPICKUPDEPOT_LAUNCHPOSE:
                follower.setMaxPowerScaling(1);
                if (!pathStarted) {


                    follower.followPath(endPickupDepot_LaunchPose, true);
                    pathStarted = true;
                }


                // Launch automatically when at launchPose
                LaunchArtifacts(launchPose);


                if (!follower.isBusy() && launchComplete()) {
                    setPathState(PathState.LAUNCHPOSE_LEAVE);
                    pathStarted = false;
                }
                break;


            case LAUNCHPOSE_LEAVE:
                if (!pathStarted) {
                    follower.followPath(launchPose_Leave, true);
                    pathStarted = true;
                }


                if (!follower.isBusy()) {
                    pathStarted = false;
                    requestOpModeStop();
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

        boolean feeding =
                launcherState == LauncherState.FEEDING;

        if (feeding || isIntakingState()) {
            indexer.setPower(1);
        } else {
            indexer.setPower(0);
        }
    }
}
