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
    public class DriveForwardAndStop extends OpMode{

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

            Launcher.setVelocityPIDFCoefficients(100, 0, 0, 12.6);
            Launcher2.setVelocityPIDFCoefficients(100,0 ,0 , 12.6); //p20 f12.6

        }

        //-----------------Launcher Logic-------------------------\\
        org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.IDLE;
        Timer launcherTimer = new Timer();

        void LaunchArtifacts(Pose targetPose) {
            // Start launcher if robot is within tolerance
            telemetry.addLine("thinking about launching");
            if (launcherState == org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.IDLE) {
                Pose current = follower.getPose();
                double dx = Math.abs(current.getX() - targetPose.getX());
                double dy = Math.abs(current.getY() - targetPose.getY());
                double dHeading = Math.abs(current.getHeading() - targetPose.getHeading());

                if (dx < POSITION_TOLERANCE && dy < POSITION_TOLERANCE && dHeading < HEADING_TOLERANCE) {
                    telemetry.addLine("Im launching now I decided");
                    launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.WAIT_FOR_FLYWHEEL;
                    launcherTimer.resetTimer();
                }
            }
        }
        // Launch tolerance
        double POSITION_TOLERANCE = 2.0; // +/- in inches
        double HEADING_TOLERANCE = Math.toRadians(5); // +/- in degrees

        void updateLauncher() {
            switch (launcherState) {

                case IDLE:
                    break;

                case WAIT_FOR_FLYWHEEL:
                    startFlywheel();
                    if (flywheelAtSpeed()) {
                        launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.OPEN_GATE;
                        launcherTimer.resetTimer();
                    }
                    break;

                case OPEN_GATE:
                    forkservo.setPosition(0.9);
                    forkservo2.setPosition(0.6);
                    if (launcherTimer.getElapsedTimeSeconds() > 0.5) {
                        launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.FEEDING;
                        launcherTimer.resetTimer();
                    }
                    break;

                case FEEDING:
                    indexer.setPower(1);
                    if (launcherTimer.getElapsedTimeSeconds() > 1.0) {
                        indexer.setPower(0);
                        launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.CLOSING;
                    }
                    break;

                case CLOSING:
                    forkservo.setPosition(0.65);
                    forkservo2.setPosition(0.85);
                    launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.DONE;
                    break;

                case DONE:
                    stopFlywheel();
                    launcherState = launcherState.IDLE;
                    break;
            }
        }

        void resetLauncher() {
            launcherState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.IDLE;
        }

        void startFlywheel() {
            Launcher.setVelocity(1190);
            Launcher2.setVelocity(1190);
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
            return launcherState == org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.LauncherState.DONE;
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



        private Follower follower;
        private Timer pathTimer, opModeTimer;

        //=======================================PATHING STUFF========================================\\
        public enum PathState {
            STARTPOSE_ENDPOSE,
        }

        org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.PathState pathState;

        private final Pose startPose = new Pose(0, 0, Math.toRadians(90));//START POSEprivate final Pose endPickupSpike3 = new Pose(13.686872586872596, 34.42857142857139, Math.toRadians(0));//PICKUP 2ND SPIKE
        private final Pose endPose = new Pose(0, 15, Math.toRadians(90));//LAUNCH FINAL 3 ARTIFACTS + LEAVE

        private PathChain
                startPose_endPose;

        public void buildPaths() {

            // Move from start to launch pose
            startPose_endPose = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                    .build();
        }

        boolean pathStarted = false;
        public void statePathUpdate() {
            switch (pathState) {

                case STARTPOSE_LAUNCHPOSE:
                    // Start path if not already started
                    if (!pathStarted) {
                        follower.followPath(startPose_endPose, true);
                        pathStarted = true;

                        // Move to next path only when both path and launcher complete
                        if (!follower.isBusy() && launchComplete()) {

                            pathStarted = false;
                        }
                        break;
                    }

                        default:
                            telemetry.addLine("No State Commanded");
                            break;

            }
        }

        public void setPathState(org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.PathState newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }

        @Override
        public void init() {
            initHardware();

            pathState = org.firstinspires.ftc.teamcode.pedroPathing.BlueAuto.PathState.STARTPOSE_LAUNCHPOSE;
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


