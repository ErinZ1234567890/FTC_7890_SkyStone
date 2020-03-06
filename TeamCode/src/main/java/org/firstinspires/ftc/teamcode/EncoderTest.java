package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;

/*
7890 Space Lions 2019 "FULL AUTO PARKBLU"
author: 7890 Software (TEAM MEMBERS)
GOALS: (GOALS)
DESCRIPTION: This code is used for our autonomous when we are located on the side of the tray
 */
@Autonomous(name="encoderbois", group="Iterative Opmode")
public class EncoderTest extends OpMode
{

    /*
    ---MOTORS---
     */
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armMotor;

    /*
    ---STATES---
     */
    EncoderState encoderDrive;





    ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
    ArrayList<ModernRoboticsI2cRangeSensor> mrrs = new ArrayList<ModernRoboticsI2cRangeSensor>();


    public void init() {



        /*
        ---HARDWARE MAP---
         */
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");



        /*
        ---MOTOR DIRECTIONS---
         */
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        /*
        ---GROUPING---
         */
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        //mrrs.add(distanceSensor);

        /*
        ---USING STATES---
         */
        encoderDrive = new EncoderState(motors, 15, -0.3);

        encoderDrive.setNextState(null);

    }


    @Override
    public void start(){
        /*
        telemetry.addData("target " , encoderDrive.getTarget());
        telemetry.update();

         */
        telemetry.addData("pos " , encoderDrive.getPos());
        telemetry.update();
        machine = new StateMachine(encoderDrive);
    }


    private StateMachine machine;
    public void loop()  {

        /*
       telemetry.addData("x ", encoderDrive.getX());
       telemetry.update();
         */
        telemetry.addData("pos " , encoderDrive.getPos());
        telemetry.update();
        machine.update();

    }


//    @Override
//    public void stop() {
//    }

}


