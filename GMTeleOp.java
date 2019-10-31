/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="GreenMachine TeleOp", group="GreenwoodFTC")
@Disabled
public class GMTeleOp extends OpMode{

    /*declaring our robot as an object*/
    HwGM robot = new HwGM();

    double
            left,
            right,
            gripperArmSpeed,
            middleArmSpeed;
            //GripperPosition = robot.GR_GRIPPER_IN,
            //midArmPosition = robot.MID_ARM_UP;
	
	final double
            GRIPPER_ARM_SPEED_FACTOR = 0.45,
	        MID_ARM_SPEED_FACTOR = 0.45;
   

    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */
    @Override
    public void init() {

        robot.init(hardwareMap); // This will initialize the robot object via it's init method

        //this will send a telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code will run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     *this code will run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code will run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

	//==========================================================
	//						GamePad One
	//==========================================================
		/*
		Controls-
			BallCollecter-	L/R Triggers
			DriveTrain- L/R Joysticks
		*/
	
        // This controls the drive train
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.frontLeftDrive.setPower(left);
        robot.frontRightDrive.setPower(right);

        
		//Controls Gripper

        /* BUG - "gripperArm not defined as a motor. Also, constants GR_ARM_IN and GR_ARM_OUT are not defined.
        if (gamepad1.left_trigger >= 0.10) {
            robot.gripperArm.setTargetPosition(robot.GR_ARM_IN);
            gripperArmSpeed = gamepad1.left_trigger * GRIPPER_ARM_SPEED_FACTOR;
            robot.gripperArm.setPower(gripperArmSpeed);
        }

        if (gamepad1.right_trigger >= 0.10) {
            robot.gripperArm.setTargetPosition(robot.GR_ARM_OUT);
            gripperArmSpeed = gamepad1.right_trigger * GRIPPER_ARM_SPEED_FACTOR;
            robot.gripperArm.setPower(gripperArmSpeed);
        }

        if (gamepad1.left_trigger < 0.10 && gamepad1.right_trigger < 0.10)
            robot.gripperArm.setPower(0);
        */

  	//==========================================================
	//						GamePad Two
	//==========================================================
		/*
		Controls-
			Middle Arm-	L/R Triggers
			Base Arm- L/R Joysticks
		*/
        // This controls the motor at the base of the arm
		//CHANGE TO SYSTEM SIMULAR TO TRIGGERS
        left = gamepad2.left_stick_y;
        right = -gamepad2.right_stick_y;
        //robot.baseArm.setPower(left);
        //robot.baseArm.setPower(right);

        
		//Controls the servo at the mid point of the arm

        /* BUG - The following code uses commands for a motor, but "midArm" is defined as a servo
        if (gamepad2.left_trigger >= 0.10) {
            robot.midArm.setTargetPosition(robot.MID_ARM_UP);
            middleArmSpeed = gamepad2.left_trigger * MID_ARM_SPEED_FACTOR;
            robot.midArm.setPower(middleArmSpeed);
        }

        if (gamepad2.right_trigger >= 0.10) {
            robot.midArm.setTargetPosition(robot.MID_ARM_DOWN);
            middleArmSpeed = gamepad2.right_trigger * MID_ARM_SPEED_FACTOR;
            robot.midArm.setPower(middleArmSpeed);
        }

        if (gamepad2.left_trigger < 0.10 && gamepad2.right_trigger < 0.10)
            robot.midArm.setPower(0);
         */
		
	//==========================================================
	//						Telemetry
	//==========================================================
		
		//EDIT LATER NOT UP TO DATE
		
        // this telemetry will report power setpoint to the Driver Station phone
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        // telemetry.addData("G Gripper", glyphGripperPosition);  // BUG - glyphGripperPosition is not declared and defined
        telemetry.addData("Left Bumper", gamepad1.left_bumper);
        telemetry.addData("Right Bumper", gamepad1.right_bumper);
        // telemetry.addData("G Arm Position", robot.glyphArm.getCurrentPosition());  // BUG - glyphArm is not defined
    }

    /*
     * Code will run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Sets all motors to zero power
        robot.frontLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);

    }

}
