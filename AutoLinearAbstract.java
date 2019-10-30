/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Abstract Linear Autonomous OpMode
 * Created 19Nov2017 for the Greenwood FTC Robotics Club.
 * This class provides relicRobot status and methods typical to all Relic Game autonomous OpModes
 *
 * Revised 18Dec2017 - Center Grove competition adjustments
 * Revised 23Jan2018 - Add JewelArm2 servo
 * Revised 13Feb2018 - Modified Glyph Arm gear ratio and speed
 * Revised 15Feb2018 - Increased servo speed increments
 *                   - Added servoRelicLinear and initialization logic*/

//@Disabled
public abstract class AutoLinearAbstract extends LinearOpMode {

    /* =======================================================
     * CLASS MEMBERS (i.e., Class Status)
     * Common autonomous opmode members
     * ======================================================= */

    /* -------------------------------------------------------
     * Public (Shared) Class Members
     * Automated objects, timers, variables, constants
     * ------------------------------------------------------- */

    // OBJECTS
    mecanumDrive
            driveTrain;

    DeviceTargetMotor
            boxMover,
            scissorLift;

    DeviceTargetServo
            scissorTop,
            scissorSides,
            clawServoLeft,
            clawServoRight;



    ElapsedTime
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            autoTimer = new ElapsedTime();    // Autonomous timer


    // CONSTANTS
    //static final int


    final static double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = 0.25,

            DRIVE_TRAIN_PIVOT_SPEED = 0.2,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.5,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.6,

            SCISSOR_TOP_UP = 0.8,
            SCISSOR_TOP_DOWN = 0,
            SCISSOR_SIDES_OUT = 0,
            SCISSOR_SIDES_IN = 0.8,
            BOX_MOVER_OUT = 3000,
            BOX_MOVER_IN = 0,
            BOX_MOVER_INIT_POS = 0.50,
            CLAW_SERVO_LEFT_UP = 0.65,
            CLAW_SERVO_LEFT_DOWN = 0.05,
            CLAW_SERVO_RIGHT_UP = 0.05,
            CLAW_SERVO_RIGHT_DOWN = 0.65;


    final static boolean
            FORWARD = false,
            REVERSE = true;



    /* =======================================================
     * CLASS METHODS (i.e., Class Behavior)
     * ======================================================= */

    /* -------------------------------------------------------
     * Method: runOpMode (Overridden Linear OpMode)
     * Purpose: Establish and initialize automated objects, and signal initialization completion
     * ------------------------------------------------------- */
    @Override
    public void runOpMode() {

        /* INITIALIZE ROBOT - ESTABLISH ROBOT OBJECTS */

        // Noti.update();

        /* Drive Train constructor: hardwareMap, left motor name, left motor direction, right motor name, right motor direction,
                                    encoder counts per output shaft revolution, gear ratio, wheel radius */
        driveTrain = new mecanumDrive(hardwareMap,"front_left_drive",FORWARD,"front_right_drive",REVERSE,"rear_left_drive",FORWARD ,"front_right_drive",REVERSE,1120,1.0,2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */
        scissorLift = new DeviceTargetMotor(hardwareMap,"scissor_lift",REVERSE,1680,13*24/16);
        boxMover = new DeviceTargetMotor(hardwareMap, "box_mover", REVERSE, 1440);
        /* Color sensor constructor: hardwareMap, sensor name, sensor I2C address */
        // colorLeftJewel = new DeviceColorSensor(hardwareMap,"left_jewel_color",0x3c);
        //   colorRightJewel = new DeviceColorSensor(hardwareMap,"right_jewel_color",0x30);

        /* Target-Servo constructor: hardwareMap, servo name, initial servo position */
        scissorTop = new DeviceTargetServo(hardwareMap,"scissor_top", SCISSOR_TOP_UP);
        scissorSides = new DeviceTargetServo(hardwareMap,"scissor_sides", SCISSOR_SIDES_OUT);
        clawServoLeft = new DeviceTargetServo(hardwareMap, "claw_servo", CLAW_SERVO_LEFT_UP);
        /* INITIALIZE ROBOT - INITIALIZE ROBOT OBJECTS AND CLASSES*/


        // Notify drive station that robot objects are being initialized
        telemetry.addLine("Wait - Initializing Robot Objects");
        telemetry.update();

         /* Reset encoders and place motors into the 'Run-to-Position' mode
            Note: The initialization calls in the following methods could not be performed in the respective
           object constructors */
        driveTrain.resetEncoders();
        scissorLift.resetEncoder();
        boxMover.resetEncoder();

        /* Lock drive train at current position */
        driveTrain.front.motorLeft.goToAbsoluteDistance(driveTrain.front.motorLeft.getPosition(),DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.front.motorRight.goToAbsoluteDistance(driveTrain.front.motorRight.getPosition(),DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorLeft.goToAbsoluteDistance(driveTrain.rear.motorLeft.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);
        driveTrain.rear.motorRight.goToAbsoluteDistance(driveTrain.rear.motorRight.getPosition(), DRIVE_TRAIN_DEFAULT_SPEED);



        // Note: Servo initialization is completed in the respective object constructors


        /* INITIALIZE ROBOT - SIGNAL INITIALIZATION COMPLETE */

        // Notify drive station that initialization is wrapping up
        telemetry.addLine("Wait - Initializing Almost Complete");
        telemetry.update();





        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();


        // WAIT FOR THE GAME TO START (driver presses PLAY)
        waitForStart();

        autoTimer.reset();  // Reset/restart the autotimer



        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }



    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */
    void driveTrainTelemetry () {
        telemetry.addLine();
        telemetry.addLine("Left Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.front.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.front.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.front.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.front.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.front.leftSpeed);


        telemetry.addLine("Left Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.rear.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.rear.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.rear.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.rear.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.rear.leftSpeed);

        telemetry.addLine();
        telemetry.addLine("Right Drive Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.front.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.front.motorRight.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.front.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.front.motorRight.targetCount);
        telemetry.addData("  Is Busy",driveTrain.front.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.front. rightSpeed);

        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.rear.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.rear.motorRight.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.rear.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.rear.motorRight.targetCount);
        telemetry.addData("  Is Busy",driveTrain.rear.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.rear. rightSpeed);
    }

    void motorTelemetryDegrees (DeviceTargetMotor motor) {
        telemetry.addLine();
        telemetry.addLine(motor.name);
        telemetry.addData(" Position in Degrees", "%.2f degrees ", motor.getDegrees());
        telemetry.addData(" Position in Counts", motor.targetMotor.getCurrentPosition());
    }
        boolean Kill ( double autoTime) {
            boolean eStop;
            if(!opModeIsActive()) {

                driveTrain.stop();
                scissorLift.stop();

                eStop = true;

            }
            else
                eStop = false;

            return eStop || autoTimer.seconds()>= autoTime;



        }







    }




