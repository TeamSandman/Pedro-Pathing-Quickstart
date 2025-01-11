package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawPrototype {

    private Servo Pincer_Left;
    private Servo Pincer_Right;
    private Servo Specimen_Claw;
    private Servo   Closer_Servo;
    private Servo Closer_Servo_2;
    private Servo Rotater_Servo;
    private Servo Sample_Grabber;
    private Servo Sample_Grabber_Arm;
    private CRServo Intake_Left;
    private CRServo Intake_Right;


    private CRServo Spinner;


    public void initPincer_Claw(HardwareMap hwMap) {


        Pincer_Left = hwMap.get(Servo.class, "Pincer_Left");
        Pincer_Right = hwMap.get(Servo.class, "Pincer_Right");

    }

    public void initSpecimen_Claw(HardwareMap hwMap) {


        Specimen_Claw = hwMap.get(Servo.class, "Specimen_Claw");
        Closer_Servo_2 = hwMap.get(Servo.class, "Closer_Servo_2");
        Closer_Servo = hwMap.get(Servo.class,  "Closer_Servo");

    }
    public void initSampleGrabbers(HardwareMap hwMap) {
        Sample_Grabber = hwMap.get(Servo.class, "Sample_Grabber");
        Sample_Grabber_Arm = hwMap.get(Servo.class, "Sample_Grabber_Arm");
    }
    public void initIntake_Claw(HardwareMap hwMap) {


        Rotater_Servo= hwMap.get(Servo.class, "Rotater_Servo");
        Intake_Left = hwMap.get(CRServo .class, "Intake_Left");
        Intake_Right =hwMap.get(CRServo.class, "Intake_Right");


    }
    public void initR_30Hrs(HardwareMap hwMap) {


        Spinner = hwMap.get(CRServo.class, "Spinner");

    }

    public void pincerClaw(double input){
        Pincer_Right.setPosition(1-input);
        Pincer_Left.setPosition(input);
    }



    public void Closer_Servo_Closed(){

        Closer_Servo.setPosition(.25);
        Closer_Servo_2.setPosition(.75);
    }
    public void Closer_Servo_Open(){

        Closer_Servo.setPosition(.6);
        Closer_Servo_2.setPosition(.4);
    }
    public void Grabber_Arm(){
        Sample_Grabber.setPosition(0);
    }
    public void Grabber_Arm_Extend(){
        Sample_Grabber.setPosition(0.75);
    }
    public void Sample_Grabber(){
        Sample_Grabber_Arm.setPosition(.75); //dpad down
    }
    public void Sample_Grabber_Closed(){
        Sample_Grabber_Arm.setPosition(1);//was 1: dpad up
    }
    public void Specimen_Claw_Open(){

        Specimen_Claw.setPosition(.5); // was .8
    }
    public void Specimen_Claw_Closed(){

        Specimen_Claw.setPosition(0.15); //was .53
    }
    public void intakeClaw (double left, double right){
        Intake_Left.setPower(-left);
        Intake_Right.setPower(right);

    }
    public void rotateIntake (double degree){
        Rotater_Servo.setPosition(degree);

    }

    public  void spinner (double spin) {
        Spinner.setPower(spin);

    }

}

