package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.StateMachine.State;

import java.util.ArrayList;

/*
7890 Space Lions 2019 "DISTANCEMOVESTATE"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
DESCRIPTION: This code is a distance move state, used to simplify our autonomous code.
It simply moves the robot according to the directions in our autonomous.
 */
public class EncoderState implements State{
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    State NextState;
    double dist;
    String turn = "null";
    boolean isMoved = false;
    double power;
    boolean x = false;
    int target;

    //Setting up encoder variables
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public EncoderState(ArrayList<DcMotor> motor, double distance, double speed){
        leftFront = motor.get(0);
        rightFront = motor.get(1);
        leftBack = motor.get(2);
        rightBack = motor.get(3);
        dist = distance;
        power  = speed;
    }

    public void setNextState(State state) {
        NextState  = state;

    }

    public void start(){
        x = true;

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public State update(){
        /*
        move("forward", power);
        wait(time);
        */
        encoderDrive(15);
        stop(leftFront,rightFront,leftBack,rightBack);
        return NextState;
    }

    public void encoderDrive(double timeout) {
        target = leftFront.getCurrentPosition() + (int)(dist*COUNTS_PER_INCH);

        leftFront.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightBack.setTargetPosition(target);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while(runtime.seconds() < timeout
                && leftFront.isBusy()
                && rightFront.isBusy()
                && leftBack.isBusy()
                && rightBack.isBusy()) {
            //oogabooga
        }

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }

    public boolean getX() {
        return x;
    }
    public int getTarget() {return target;}
    public int getPos() {return leftFront.getCurrentPosition();}



}
