package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "J26_100A_FarBlue", group = "Auto",preselectTeleOp = "26-75.5T-Full Control")
public class J26_100A_FarBlue extends LinearOpMode {

    /* ================= CONFIG ================= */
    static final DcMotorEx.Direction LEFT_FRONT_DIR  = DcMotorEx.Direction.REVERSE;
    static final DcMotorEx.Direction RIGHT_FRONT_DIR = DcMotorEx.Direction.FORWARD;
    static final DcMotorEx.Direction LEFT_REAR_DIR   = DcMotorEx.Direction.REVERSE;
    static final DcMotorEx.Direction RIGHT_REAR_DIR  = DcMotorEx.Direction.REVERSE;

    static final double LEFT_INTAKE_DIR  =  1.0;
    static final double RIGHT_INTAKE_DIR = -1.0;

    /* ================= DRIVE & ROTATION TUNING ================= */
    static final double DRIVE_TICKS_PER_REV = 537.6; // GoBilda Yellowjacket
    static final double WHEEL_DIAMETER_IN = 4.0;
    static final double DRIVE_CORRECTION = 0.91;
    static final int DRIVE_TOLERANCE_TICKS = (int)((0.5) * DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN));

    static final double ROTATE_TICKS_PER_DEGREE = 10.0 * 0.75;
    static final double ROTATE_TOLERANCE_DEG = 1.5;

    static final double MAX_DRIVE_POWER = 0.9;
    static final double MIN_DRIVE_POWER = 0.15;
    static final double MAX_ROTATE_POWER = 0.6;
    static final double MIN_ROTATE_POWER = 0.15;

    // ACCEL/DECEL FACTORS
    static final double ACCEL_FACTOR = 0.9;  // 1 = max accel, 0 = very slow
    static final double DECEL_FACTOR = 0.9;  // 1 = abrupt stop, 0 = very slow decel

    /* ================= LAUNCH TUNING ================= */
    static final double LAUNCH_RPM = 3300;
    static final int LAUNCH_CYCLES = 3;
    static final double LAUNCHER_TICKS_PER_REV = 28;
    static final double INDEXER_TICKS_PER_REV = 288; //288
    static final double INDEXER_POWER = 0.4;
    static final int INDEXER_DEGREES = 200;
    static final double FORK_UP = 0.78;
    static final double FORK_DOWN = 0.25;
    static final int FORK_MOVE_DELAY_MS = 150;

    /* ================= INTAKE ================= */
    static final double INTAKE_MOTOR_POWER = -0.6;
    static final double INTAKE_SERVO_POWER = 1.0;

    /* ================= HARDWARE ================= */
    DcMotorEx LeftFront, RightFront, LeftRear, RightRear;
    DcMotorEx Launcher, IntakeMotor, indexer;
    CRServo LeftIntake, RightIntake;
    Servo forkservo;

    /* ================= STEP SYSTEM ================= */
    enum StepType { MOVE, ROTATE, LAUNCH, INTAKE_ON, INTAKE_OFF }

    static class Step {
        StepType type;
        double fwd, str, speed;
        Step(StepType t, double fwd, double str, double speed) {
            type = t; this.fwd = fwd; this.str = str; this.speed = speed;
        }
        static Step move(double fwd, double str, double speed) { return new Step(StepType.MOVE,fwd,str,speed); }
        static Step rotate(double deg) { return new Step(StepType.ROTATE,deg,0,0); }
        static Step launch() { return new Step(StepType.LAUNCH,0,0,0); }
        static Step intakeOn() { return new Step(StepType.INTAKE_ON,0,0,0); }
        static Step intakeOff() { return new Step(StepType.INTAKE_OFF,0,0,0); }
    }

    List<Step> steps = new ArrayList<>();

    @Override
    public void runOpMode() {

        initHardware();
        buildPath();

        waitForStart();
        spinUpLauncher();

        for (Step s : steps) {
            if (!opModeIsActive()) break;
            executeStep(s);
        }

        stopDrive();
    }

    /* ================= PATH ================= */
    void buildPath() {
        //preloads
        steps.add(Step.move(3,0,0.6));
        steps.add(Step.rotate(-20));
        steps.add(Step.launch());
        steps.add(Step.intakeOff());
        steps.add(Step.rotate(20));
        
        //1st spike mark
        steps.add(Step.move(24,0,0.6));
        steps.add(Step.rotate(92));
        steps.add(Step.intakeOn());
        steps.add(Step.move(-48,0,0.5));
        
        steps.add(Step.move(48,0,0.5));
        steps.add(Step.rotate(-92));
        steps.add(Step.move(-23,0,0.3));
        steps.add(Step.rotate(-20));
        steps.add(Step.launch());
        steps.add(Step.intakeOff());
        steps.add(Step.move(12,0,0.6));
    }

    void executeStep(Step s) {
        switch (s.type) {
            case MOVE: driveEncoders(s.fwd,s.str,s.speed); break;
            case ROTATE: rotateEncoders(s.fwd); break;
            case LAUNCH: launchTriple(); break;
            case INTAKE_ON: intakeOn(); break;
            case INTAKE_OFF: intakeOff(); break;
        }
    }

    /* ================= DRIVE WITH TUNABLE ACCEL/DECEL ================= */
    void driveEncoders(double fwdInches, double strInches, double maxPower) {
        int targetFL = (int)((fwdInches + strInches) * DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN) * DRIVE_CORRECTION);
        int targetFR = (int)((fwdInches - strInches) * DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN) * DRIVE_CORRECTION);
        int targetBL = (int)((fwdInches - strInches) * DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN) * DRIVE_CORRECTION);
        int targetBR = (int)((fwdInches + strInches) * DRIVE_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN) * DRIVE_CORRECTION);

        LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + targetFL);
        RightFront.setTargetPosition(RightFront.getCurrentPosition() + targetFR);
        LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + targetBL);
        RightRear.setTargetPosition(RightRear.getCurrentPosition() + targetBR);

        LeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double currentPower = 0;

        while(opModeIsActive() && isBusy()) {
            int remFL = LeftFront.getTargetPosition() - LeftFront.getCurrentPosition();
            int remFR = RightFront.getTargetPosition() - RightFront.getCurrentPosition();
            int remBL = LeftRear.getTargetPosition() - LeftRear.getCurrentPosition();
            int remBR = RightRear.getTargetPosition() - RightRear.getCurrentPosition();

            int maxRem = Math.max(Math.max(Math.abs(remFL), Math.abs(remFR)), Math.max(Math.abs(remBL), Math.abs(remBR)));

            // Acceleration
            currentPower = Math.min(currentPower + ACCEL_FACTOR*0.05, maxPower);

            // Deceleration
            double decelScale = Math.pow((double)maxRem/400.0, DECEL_FACTOR);
            double powerScaled = Math.max(MIN_DRIVE_POWER, Math.min(currentPower, currentPower*decelScale));

            double pFL = Math.signum(remFL) * powerScaled;
            double pFR = Math.signum(remFR) * powerScaled;
            double pBL = Math.signum(remBL) * powerScaled;
            double pBR = Math.signum(remBR) * powerScaled;

            LeftFront.setPower(pFL);
            RightFront.setPower(pFR);
            LeftRear.setPower(pBL);
            RightRear.setPower(pBR);
        }
        stopDrive();
    }

    /* ================= ROTATE WITH TUNABLE ACCEL/DECEL ================= */
    void rotateEncoders(double degrees) {
        int ticks = (int)(degrees * ROTATE_TICKS_PER_DEGREE);

        LeftFront.setTargetPosition(LeftFront.getCurrentPosition() + ticks);
        LeftRear.setTargetPosition(LeftRear.getCurrentPosition() + ticks);
        RightFront.setTargetPosition(RightFront.getCurrentPosition() - ticks);
        RightRear.setTargetPosition(RightRear.getCurrentPosition() - ticks);

        LeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double currentPower = 0;

        while(opModeIsActive() && isBusyRotation()) {
            int remFL = LeftFront.getTargetPosition() - LeftFront.getCurrentPosition();
            int remFR = RightFront.getTargetPosition() - RightFront.getCurrentPosition();
            int remBL = LeftRear.getTargetPosition() - LeftRear.getCurrentPosition();
            int remBR = RightRear.getTargetPosition() - RightRear.getCurrentPosition();

            int maxRem = Math.max(Math.max(Math.abs(remFL), Math.abs(remFR)), Math.max(Math.abs(remBL), Math.abs(remBR)));

            currentPower = Math.min(currentPower + ACCEL_FACTOR*0.05, MAX_ROTATE_POWER);
            double decelScale = Math.pow((double)maxRem/(ROTATE_TICKS_PER_DEGREE*30.0), DECEL_FACTOR);
            double powerScaled = Math.max(MIN_ROTATE_POWER, Math.min(currentPower, currentPower*decelScale));

            double pFL = Math.signum(remFL) * powerScaled;
            double pFR = Math.signum(remFR) * powerScaled;
            double pBL = Math.signum(remBL) * powerScaled;
            double pBR = Math.signum(remBR) * powerScaled;

            LeftFront.setPower(pFL);
            LeftRear.setPower(pBL);
            RightFront.setPower(-pFR);
            RightRear.setPower(-pBR);
        }
        stopDrive();
    }

    boolean isBusy() {
        return !(Math.abs(LeftFront.getTargetPosition() - LeftFront.getCurrentPosition()) < DRIVE_TOLERANCE_TICKS &&
                 Math.abs(RightFront.getTargetPosition() - RightFront.getCurrentPosition()) < DRIVE_TOLERANCE_TICKS &&
                 Math.abs(LeftRear.getTargetPosition() - LeftRear.getCurrentPosition()) < DRIVE_TOLERANCE_TICKS &&
                 Math.abs(RightRear.getTargetPosition() - RightRear.getCurrentPosition()) < DRIVE_TOLERANCE_TICKS);
    }

    boolean isBusyRotation() {
        int tol = (int)(ROTATE_TICKS_PER_DEGREE * ROTATE_TOLERANCE_DEG);
        return !(Math.abs(LeftFront.getTargetPosition() - LeftFront.getCurrentPosition()) < tol &&
                 Math.abs(RightFront.getTargetPosition() - RightFront.getCurrentPosition()) < tol &&
                 Math.abs(LeftRear.getTargetPosition() - LeftRear.getCurrentPosition()) < tol &&
                 Math.abs(RightRear.getTargetPosition() - RightRear.getCurrentPosition()) < tol);
    }

    /* ================= LAUNCH ================= */
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
    }
    double getLauncherRPM() {
        return Launcher.getVelocity()*60.0/LAUNCHER_TICKS_PER_REV;
    }

    void intakeOn() {
        IntakeMotor.setPower(INTAKE_MOTOR_POWER);
        LeftIntake.setPower(INTAKE_SERVO_POWER*LEFT_INTAKE_DIR);
        RightIntake.setPower(INTAKE_SERVO_POWER*RIGHT_INTAKE_DIR);
        
        // Turn indexer on as well
        indexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        indexer.setPower(INDEXER_POWER);  // use the same INDEXER_POWER you already have
    }

    void intakeOff() {
        IntakeMotor.setPower(0);
        LeftIntake.setPower(0);
        RightIntake.setPower(0);
    }

    void stopDrive() {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
        
        // Stop indexer as well
        indexer.setPower(0);
    }

    void initHardware() {
        LeftFront = hardwareMap.get(DcMotorEx.class,"LeftFront");
        RightFront = hardwareMap.get(DcMotorEx.class,"RightFront");
        LeftRear = hardwareMap.get(DcMotorEx.class,"LeftRear");
        RightRear = hardwareMap.get(DcMotorEx.class,"RightRear");
        Launcher = hardwareMap.get(DcMotorEx.class,"Launcher");
        IntakeMotor = hardwareMap.get(DcMotorEx.class,"IntakeMotor");
        indexer = hardwareMap.get(DcMotorEx.class,"indexer");
        LeftIntake = hardwareMap.get(CRServo.class,"LeftIntake");
        RightIntake = hardwareMap.get(CRServo.class,"RightIntake");
        forkservo = hardwareMap.get(Servo.class,"forkservo");

        LeftFront.setDirection(LEFT_FRONT_DIR);
        RightFront.setDirection(RIGHT_FRONT_DIR);
        LeftRear.setDirection(LEFT_REAR_DIR);
        RightRear.setDirection(RIGHT_REAR_DIR);

        indexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
