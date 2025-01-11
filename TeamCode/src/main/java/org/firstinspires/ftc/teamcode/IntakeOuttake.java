package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeOuttake {


// Four Bar Servos (Control Hub Ports 2 [FourBarLeft], 3 [FourBarRight], 4[FourBarPitch]
    private Servo FourBarLeft;

    private Servo FourBarRight;

    private Servo FourBarPitch;


// Intake Servos (Control Hub Ports 0 [IntakeLeft], 1 [IntakeRight])
    private CRServo IntakeLeft;

    private CRServo IntakeRight;


// Horizontal Slides Motor (Expansion Hub Port 0)
    private DcMotorEx HorizontalSlides;


// Outtake Claw (Expansion Hub Ports 0 [ClawLeft], 1 [ClawRight], 2 [OuttakeArm])
    private Servo ClawLeft;

    private Servo ClawRight;

    private Servo OuttakeArm;



//Vertical Slides Motor ( Expansion Hub Port 1)
    private DcMotorEx VerticalSlides;



    public void initIntake(HardwareMap hwMap){
        FourBarLeft = hwMap.get(Servo.class, "FourBarLeft");
        FourBarRight = hwMap.get(Servo.class, "FourBarRight");
        FourBarPitch = hwMap.get(Servo.class, "FourBarPitch");

        IntakeLeft = hwMap.get(CRServo.class, "IntakeLeft");
        IntakeRight = hwMap.get(CRServo.class, "IntakeRight");

        HorizontalSlides = hwMap.get(DcMotorEx.class, "HorizontalSlides");

        HorizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HorizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HorizontalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
//Intake Sample
    public void intakeIn(){
       IntakeLeft.setPower(1);
       IntakeRight.setPower(-1);
    }
//Spit out Sample
    public void intakeOut(){
        IntakeLeft.setPower(-.25-.1);
        IntakeRight.setPower(.25+.1);
    }
//Turn intake off
    public void intakeOff(){
        IntakeLeft.setPower(0);
        IntakeRight.setPower(0);
    }
//Lower four bar to intake sample
    public void fourBarDown(){
        FourBarLeft.setPosition(0);//was.09
        FourBarRight.setPosition(1);//was.47
    }
//Extend four bar to search for sample
    public void fourBarSearch(){
        FourBarLeft.setPosition(.1+.05);//was .19
        FourBarRight.setPosition(.9-.05);//was .37
    }

    public void fourBarTransfer(){
        FourBarLeft.setPosition(.2+.025);
        FourBarRight.setPosition(.8-.025);
    }

//Retract four bar
    public void fourBarStowed(){
      FourBarLeft.setPosition(.26);//was .35
      FourBarRight.setPosition(.74);//was .2
    }


    public void fourBarPitchSearch(){
        FourBarPitch.setPosition(.2-.03);
    }

    public void fourBarPitchStowed(){
        FourBarPitch.setPosition(.65);
    }
    public void fourBarPitchTransfer(){
        FourBarPitch.setPosition(.75-.05);
    }

    public void horizontalSlidesHome(){
        HorizontalSlides.setTargetPosition(0);
        HorizontalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HorizontalSlides.setPower(.35);
    }

    public int horizontalSlidesReturnPosition(){
        return HorizontalSlides.getCurrentPosition();
    }
    public void horizontalSlidesManualControl(double speed){
        HorizontalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HorizontalSlides.setPower(speed);
    }

    public void recalibrateHorizontalSlide(){
        HorizontalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initOuttake(HardwareMap hwMap){
       ClawLeft = hwMap.get(Servo.class, "ClawLeft");
       ClawRight = hwMap.get(Servo.class, "ClawRight");
       OuttakeArm = hwMap.get(Servo.class, "OuttakeArm");

       VerticalSlides = hwMap.get(DcMotorEx.class, "VerticalSlides");

       VerticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       VerticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       VerticalSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void armTransfer(){
        OuttakeArm.setPosition(1);
    }

    public void armPreScore(){
        OuttakeArm.setPosition(.25);
    }

    public void armPickUp(){
        OuttakeArm.setPosition(.1575);
    }

    public void armStowed(){
        OuttakeArm.setPosition(.05);
    }

    public void clawOpen(){
        ClawLeft.setPosition(.6);
        ClawRight.setPosition(.4);
    }

    public void clawClosed(){
        ClawLeft.setPosition(.24-.05);
        ClawRight.setPosition(.76);
    }

    public void liftStowed(){
       VerticalSlides.setTargetPosition(0);
       VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       VerticalSlides.setPower(.35);
    }


    public void liftSetHighChamber(){
        VerticalSlides.setTargetPosition(-600);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.35);
    }
    public void liftTransfer(){
        VerticalSlides.setTargetPosition(-100 +60);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftLowChamber(){
        VerticalSlides.setTargetPosition(-400);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftLowBasket(){
        VerticalSlides.setTargetPosition(-600);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftHighChamber(){
        VerticalSlides.setTargetPosition(-1410);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftAutoHighBasket(){
        VerticalSlides.setTargetPosition(-3000);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftHighBasket(){
        VerticalSlides.setTargetPosition(-2800);
        VerticalSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VerticalSlides.setPower(.5);
    }

    public void liftManualControl(double speed){
        VerticalSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VerticalSlides.setPower(speed);
    }

    public void recalibrateLift(){
        VerticalSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }





}
