package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "JavaAprilTag", group = "Competition")
public class JavaAprilTag extends LinearOpMode {

    // --- Motors & Servos ---
    private DcMotor LeftFront, LeftRear, RightFront, RightRear;
    private Servo LED_Servo, forkservo;
    private IMU imu;
    private DcMotor IntakeMotor, indexer, Launcher, Launcher2;
    private CRServo RightIntake, LeftIntake, LeftBandintake, RightBandintake;

    // --- Driver inputs ---
    private float DriverForward, DriverRight, DriverClockwise, CoDriverClockwise;

    // --- Previous states for edge detection ---
    private boolean prevLeftStickButton1 = false;
    private boolean prevLeftStickButton2 = false;
    private boolean prevRightStickButton1 = false;

    // --- AprilTag Locking ---
    private boolean IsAprilTagLockActive = false;
    private int LockTagID = 24;

    private double DesiredForward = 40.0;   // feet in front of tag
    private double DesiredLateral = 0.0;    // feet lateral offset from tag
    private double DesiredRotate = 0.0;     // degrees yaw relative to tag

    private double kDistance = 0.3;         // Forward/backward P gain
    private double kLateral = 0.3;          // Lateral P gain
    private double kBearing = 0.05;         // Rotation P gain
    private double maxPower = 0.3;          // Maximum motor power for locking

    private AprilTagProcessor myAprilTagProcessor;
    private boolean IsAprilTagDetected;

    // --- Smoothing / Acceleration ---
    private float TargetForward = 0;
    private float TargetRight = 0;
    private float TargetClockwise = 0;
    private float AccelFactor = 0.5f;  // smaller = slower acceleration

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightRear = hardwareMap.get(DcMotor.class, "RightRear");
        LED_Servo = hardwareMap.get(Servo.class, "LED");
        imu = hardwareMap.get(IMU.class, "imu");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        RightIntake = hardwareMap.get(CRServo.class, "RightIntake");
        LeftIntake = hardwareMap.get(CRServo.class, "LeftIntake");
        LeftBandintake = hardwareMap.get(CRServo.class, "LeftBandintake");
        RightBandintake = hardwareMap.get(CRServo.class, "RightBandintake");
        indexer = hardwareMap.get(DcMotor.class, "indexer");
        Launcher = hardwareMap.get(DcMotor.class, "Launcher");
        Launcher2 = hardwareMap.get(DcMotor.class, "Launcher2");
        forkservo = hardwareMap.get(Servo.class, "forkservo");

        initializeMotors();
        initializeIMU();
        initializeVisionPortal();

        waitForStart();
        ElapsedTime MatchTime = new ElapsedTime();

        while (opModeIsActive()) {

            // --- Update inputs ---
            updateDriverInputs();
            updateCoDriverInputs();

            // --- Robot subsystems ---
            handleIntake();
            handleIndexerAndFork();
            handleLauncher();

            // --- Drive ---
            double yaw = getRobotYaw();

            if (IsAprilTagLockActive) {
                AprilTagLockControl();
            }

            mecanumDrive(DriverForward, DriverRight,
                    Math.abs(CoDriverClockwise) > 0 ? CoDriverClockwise : DriverClockwise);

            telemetry.update();
        }
    }

    // ---------------- DRIVER INPUTS ----------------
    private void updateDriverInputs() {
        DriverForward = -gamepad1.right_stick_y;
        DriverRight = gamepad1.right_stick_x;
        DriverClockwise = gamepad1.left_stick_x;

        // Toggle AprilTag lock
        if (gamepad1.right_stick_button && !prevRightStickButton1) {
            IsAprilTagLockActive = !IsAprilTagLockActive;
        }
        prevRightStickButton1 = gamepad1.right_stick_button;

        // Adjust desired position dynamically
        if (gamepad1.dpad_up) DesiredForward += 0.1;
        if (gamepad1.dpad_down) DesiredForward -= 0.1;
        if (gamepad1.dpad_right) DesiredLateral += 0.1;
        if (gamepad1.dpad_left) DesiredLateral -= 0.1;
        if (gamepad1.left_trigger > 0.2) DesiredRotate += 1;
        if (gamepad1.right_trigger > 0.2) DesiredRotate -= 1;
    }

    private void updateCoDriverInputs() {
        CoDriverClockwise = gamepad2.left_stick_x;
    }

    // ---------------- INTAKE & INDEXER ----------------
    private void handleIntake() {
        if (gamepad1.right_bumper) {
            IntakeMotor.setPower(-1);
            ((DcMotorEx) IntakeMotor).setVelocity(-980);
            RightIntake.setDirection(CRServo.Direction.REVERSE);
            LeftIntake.setDirection(CRServo.Direction.FORWARD);
            RightIntake.setPower(1);
            LeftIntake.setPower(1);
            LeftBandintake.setDirection(CRServo.Direction.FORWARD);
            LeftBandintake.setPower(1);
            RightBandintake.setDirection(CRServo.Direction.REVERSE);
            RightBandintake.setPower(1);
        } else if (gamepad1.left_bumper) {
            IntakeMotor.setPower(1);
            ((DcMotorEx) IntakeMotor).setVelocity(800);
            LeftIntake.setDirection(CRServo.Direction.REVERSE);
            RightIntake.setDirection(CRServo.Direction.FORWARD);
            LeftIntake.setPower(1);
            RightIntake.setPower(1);
            LeftBandintake.setDirection(CRServo.Direction.REVERSE);
            LeftBandintake.setPower(1);
            RightBandintake.setDirection(CRServo.Direction.FORWARD);
            RightBandintake.setPower(1);
        } else {
            IntakeMotor.setPower(0);
            LeftIntake.setPower(0);
            RightIntake.setPower(0);
            LeftBandintake.setPower(0);
            RightBandintake.setPower(0);
        }
    }

    private void handleIndexerAndFork() {
        if (gamepad2.triangle) forkservo.setPosition(0.75);
        else forkservo.setPosition(0.5);

        if (forkservo.getPosition() >= 0.75) indexer.setPower(1);
        else indexer.setPower(0);
    }

    private void handleLauncher() {
        boolean leftBumperPressed = gamepad2.left_bumper;
        boolean rightBumperPressed = gamepad2.right_bumper;
        boolean crossPressed = gamepad2.cross;

        if (leftBumperPressed) {
            Launcher.setDirection(DcMotor.Direction.FORWARD);
            Launcher2.setDirection(DcMotor.Direction.REVERSE);
            Launcher.setPower(1);
            Launcher2.setPower(1);
            ((DcMotorEx) Launcher).setVelocity(1540);
            ((DcMotorEx) Launcher2).setVelocity(1540);
        }
        if (rightBumperPressed) {
            Launcher.setPower(0.54);
            Launcher2.setPower(-0.54);
            ((DcMotorEx) Launcher).setVelocity(1150);
            ((DcMotorEx) Launcher2).setVelocity(1150);
        }
        if (crossPressed) {
            Launcher.setPower(0);
            Launcher2.setPower(0);
            Launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // ---------------- DRIVE ----------------
    private void mecanumDrive(double forward, double right, double clockwise) {
        double maxInput = Math.abs(forward) + Math.abs(right) + Math.abs(clockwise);
        if (maxInput > 1) {
            forward /= maxInput;
            right /= maxInput;
            clockwise /= maxInput;
        }

        LeftFront.setPower(forward + right + clockwise);
        LeftRear.setPower(forward - right + clockwise);
        RightFront.setPower(forward - right - clockwise);
        RightRear.setPower(forward + right - clockwise);
    }

    private double getRobotYaw() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double yaw = angles.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw Angle", Math.round(yaw));
        return yaw;
    }

    // ---------------- APRILTAG LOCK CONTROL ----------------
    private void AprilTagLockControl() {
        List<AprilTagDetection> detections = myAprilTagProcessor.getDetections();
        IsAprilTagDetected = false;

        double forwardError = 0;
        double lateralError = 0;
        double rotateError = 0;

        double actualDistance = 0;
        double actualLateral = 0;
        double actualBearing = 0;

        for (AprilTagDetection tag : detections) {
            if (tag.id == LockTagID) {
                IsAprilTagDetected = true;

                // Read actual tag position
                actualDistance = tag.ftcPose.range;   // feet/meters
                actualLateral = tag.ftcPose.x;        // positive = tag is left
                actualBearing = tag.ftcPose.bearing;  // degrees

                // Compute errors
                forwardError = DesiredForward - actualDistance;
                lateralError = DesiredLateral - actualLateral;
                rotateError = DesiredRotate - actualBearing;

                break;
            }
        }

        // Compute target powers
        TargetForward = (float) Range.clip(-forwardError * kDistance, -maxPower, maxPower);
        TargetRight   = (float) Range.clip(lateralError * kLateral, -maxPower, maxPower);
        TargetClockwise = (float) Range.clip(rotateError * kBearing, -maxPower, maxPower);

        // Smoothly accelerate toward target
        DriverForward   += (TargetForward - DriverForward) * AccelFactor;
        DriverRight     += (TargetRight - DriverRight) * AccelFactor;
        DriverClockwise += (TargetClockwise - DriverClockwise) * AccelFactor;

        // --- Telemetry Overlay ---
        telemetry.addData("Lock Active", IsAprilTagLockActive);
        telemetry.addData("AprilTag Detected", IsAprilTagDetected ? "YES" : "NO");

        if (IsAprilTagDetected) {
            telemetry.addData("Desired Distance (ft)", "%.2f", DesiredForward);
            telemetry.addData("Actual Distance (ft)", "%.2f", actualDistance);
            telemetry.addData("Distance Error", "%.2f", forwardError);

            telemetry.addData("Desired Lateral (ft)", "%.2f", DesiredLateral);
            telemetry.addData("Actual Lateral (ft)", "%.2f", actualLateral);
            telemetry.addData("Lateral Error", "%.2f", lateralError);

            telemetry.addData("Desired Bearing (deg)", "%.2f", DesiredRotate);
            telemetry.addData("Actual Bearing (deg)", "%.2f", actualBearing);
            telemetry.addData("Bearing Error", "%.2f", rotateError);
        }

        telemetry.addData("DriverForward Power", "%.2f", DriverForward);
        telemetry.addData("DriverRight Power", "%.2f", DriverRight);
        telemetry.addData("DriverClockwise Power", "%.2f", DriverClockwise);
    }

    // ---------------- INITIALIZATION ----------------
    private void initializeMotors() {
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initializeIMU() {
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    }

    private void initializeVisionPortal() {
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
    }
}
