package org.firstinspires.ftc.teamcode;



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




    //==========Motor Stuffs=========\\



@Autonomous
public class FlywheelTuning extends OpMode {
    float buttonCoolDown = 25;

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

    public void init() {

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
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    float P = 20;
    float F = 12.6f;
    boolean isP = true;
    float interval=1;
    int i =-1;
    public void loop(){

        if(i==-1) {
            Launcher.setVelocity(1190);
            Launcher2.setVelocity(1190);
            telemetry.addLine("beninging");
        }

        if(gamepad1.dpad_up&& i> buttonCoolDown){
            i=0;
            if(isP) {
                P = P + interval;
            }else {
                F = F + interval;
            }
        } else if (gamepad1.dpad_down &&  i>buttonCoolDown) {
            i=0;
            if(isP) {
                P = P - interval;
            }else {
                F = F - interval;
            }

        }else if (gamepad1.dpad_left &&  i>buttonCoolDown){
            interval = 0.1f;
        }else if (gamepad1.dpad_right &&  i>buttonCoolDown){
            interval = 1;
        }else if (gamepad1.a &&  i>buttonCoolDown){
            isP = !isP;
        }

        if (gamepad1.b){
            indexer.setPower(1);
        }
        else{
            indexer.setPower(0);
        }

        Launcher.setVelocityPIDFCoefficients(P, 0, 0, F);
        Launcher2.setVelocityPIDFCoefficients(P, 0, 0, F);
        telemetry.addLine("P: ["+P+"]  F: ["+F+"]");
        telemetry.addLine(String.valueOf("launcher velocities: "+Launcher.getVelocity()) + "  " + Launcher2.getVelocity());
        telemetry.addLine(String.valueOf(isP));
        telemetry.addLine(String.valueOf(interval));
        i++;
    }
}

