package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntakeOuttake;

@TeleOp
public class AssembleRobot extends OpMode {

    IntakeOuttake jamocha = new IntakeOuttake();

    @Override
    public void init(){
        jamocha.initIntake(hardwareMap);
        jamocha.fourBarDown();
        jamocha.fourBarPitchSearch();
    }
    @Override
    public void loop(){

    }
}
