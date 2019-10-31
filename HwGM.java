package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 Created 9/18/2019 by Blake Roscoe

 */
public class HwGM {

    /* Our Public OpMode Devices */
    Servo
            scissorTop =null,
            scissorSides =null,
            clawServoLeft = null,
            clawServoRight = null;

    DcMotor
            frontLeftDrive = null,
            frontRightDrive = null,
            rearLeftDrive = null,
            rearRightDrive = null,
            boxMover = null,
            scissorLift = null;

    final int
            BOX_MOVER_OUT = 4000,
            BOX_MOVER_IN = 0,
            SCISSOR_DOWN_POS = 0,
            SCISSOR_UP_POS = 9475;
    final double
            SCISSOR_TOP_UP = 0.8,
            SCISSOR_TOP_DOWN = 0,
            SCISSOR_SIDES_OUT = 0,
            SCISSOR_SIDES_IN = 0.8,
            SCISSOR_MOTOR_SPEED_FACTOR = 0.95,
            BOX_MOTOR_SPEED_FACTOR = 0.3,
            NULL_MOTOR_MOVE_SPEED = 0.1,
            CLAW_SERVO_LEFT_UP = 0.65,
            CLAW_SERVO_LEFT_DOWN = 0.05,
            CLAW_SERVO_RIGHT_UP = 0.05,
            CLAW_SERVO_RIGHT_DOWN = 0.65;

    /* The OpMode Constants */

    /* The LOCAL OpMode members. */
    HardwareMap hwMap =  null;


    /* Constructor */
    HwGM(){
    }


    /* This Initializes the standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // these are our motors and what they are called
        rearLeftDrive = hwMap.dcMotor.get("rear_left_drive");
        rearRightDrive = hwMap.dcMotor.get("rear_right_drive");
        frontLeftDrive = hwMap.dcMotor.get("front_left_drive");
        frontRightDrive = hwMap.dcMotor.get("front_right_drive");


        scissorLift = hwMap.dcMotor.get("scissor_lift");

        scissorTop = hwMap.servo.get("scissor_top");
        scissorTop.setPosition(SCISSOR_TOP_UP);

        scissorSides = hwMap.servo.get("scissor_sides");
        scissorSides.setPosition(SCISSOR_SIDES_OUT);

        clawServoLeft = hwMap.servo.get("claw_servo_left");
        clawServoLeft.setPosition(CLAW_SERVO_LEFT_UP);

        clawServoRight = hwMap.servo.get("claw_servo_right");
        clawServoRight.setPosition(CLAW_SERVO_RIGHT_UP);

        boxMover = hwMap.dcMotor.get("box_mover");

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);


        scissorLift.setDirection(DcMotor.Direction.FORWARD);
        scissorLift.setTargetPosition(SCISSOR_DOWN_POS);

        boxMover.setDirection(DcMotor.Direction.FORWARD);
        boxMover.setTargetPosition(BOX_MOVER_IN);


        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearLeftDrive.setPower(0);
        rearRightDrive.setPower(0);
        scissorLift.setPower(0);


        //Resetting encoder numbers to zero
        frontLeftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rearLeftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rearRightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //Setting motors to run without encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // other motors use encoders
        scissorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scissorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boxMover.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boxMover.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }


}