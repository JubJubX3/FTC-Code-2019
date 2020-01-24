package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GM TeleOpMecanum", group="GreenwoodFTC")  //declares the name of the class and the
// group it is in.
//@Disabled


public class GMTeleOpMecanum extends OpMode{


    /*declaring our robot as an object*/
    HwGM robot = new HwGM();


    double  //declares all double variables and their values
            //trackServoPosition = robot.TRACK_SERVO_EXTEND_SPEED,
            pullupspeed = 0;


    private boolean //declares all private booleans and their initial values (true or false)
            scissorLiftMove = false,
            bPressed = false,
            xPressed = false,
            moveBoxMover = false;

    final int
            LIFT_MAX = 6,
            LIFT_MIN = 0;


    /*
     * Code will run ONCE when the driver hits INIT
     * INIT means initialize
     */


    @Override

    public void init() { //initialization class to be used at start of tele-op

        robot.init(hardwareMap); // This will initialize the robot object via it's init method

        //this will send a telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        bPressed = false;
        xPressed = false;
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

        float FLspeed = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        float BLspeed = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        float FRspeed = -gamepad1.right_stick_y - gamepad1.right_stick_x;
        float BRspeed = -gamepad1.right_stick_y + gamepad1.right_stick_x;


        FLspeed = Range.clip(FLspeed, -1, 1);
        BLspeed = Range.clip(BLspeed, -1, 1);
        FRspeed = Range.clip(FRspeed, -1, 1);
        BRspeed = Range.clip(BRspeed, -1, 1);



        robot.rearLeftDrive.setPower(BLspeed);
        robot.rearRightDrive.setPower(BRspeed);
        robot.frontLeftDrive.setPower(FLspeed);
        robot.frontRightDrive.setPower(FRspeed);

//Controls the pull-up system of robot

        if (gamepad1.right_trigger >= 0.10) {

            robot.scissorLift.setTargetPosition(robot.SCISSOR_DOWN_POS);
            pullupspeed = gamepad1.right_trigger * robot.SCISSOR_MOTOR_SPEED_FACTOR;
            robot.scissorLift.setPower(pullupspeed);
            scissorLiftMove = true;
        }

        if (gamepad1.left_trigger >= 0.10) {

            robot.scissorLift.setTargetPosition(robot.SCISSOR_UP_POS);
            pullupspeed = gamepad1.left_trigger * robot.SCISSOR_MOTOR_SPEED_FACTOR;
            robot.scissorLift.setPower(pullupspeed);
            scissorLiftMove = true;
        }

        if (gamepad1.left_trigger < 0.10 && gamepad1.right_trigger < 0.10 && scissorLiftMove) {

            robot.scissorLift.setTargetPosition(robot.scissorLift.getCurrentPosition());
            robot.scissorLift.setPower(robot.SCISSOR_MOTOR_SPEED_FACTOR);
            scissorLiftMove = false;
        }


        if (gamepad1.dpad_up) {
            robot.boxMover.setTargetPosition(robot.BOX_MOVER_OUT);
            robot.boxMover.setPower(robot.BOX_MOTOR_SPEED_FACTOR);
            moveBoxMover = true;
        }

        if (gamepad1.dpad_down) {
            robot.boxMover.setTargetPosition(robot.BOX_MOVER_IN);
            robot.boxMover.setPower(robot.BOX_MOTOR_SPEED_FACTOR);
            moveBoxMover = true;
        }

        if (!gamepad1.dpad_up && !gamepad1.dpad_down && moveBoxMover) {
            robot.boxMover.setTargetPosition(robot.boxMover.getCurrentPosition());
            robot.boxMover.setPower(robot.NULL_MOTOR_MOVE_SPEED);
            moveBoxMover = false;
        }




        if (gamepad1.left_bumper) {
            robot.boxGrabber.setPosition(robot.BOX_GRABBER_OPEN);
        }

        if (gamepad1.right_bumper) {
            robot.boxGrabber.setPosition(robot.BOX_GRABBER_CLOSED);
        }


        if (gamepad1.y) {
            robot.clawServoLeft.setPosition(robot.CLAW_SERVO_LEFT_UP);
            robot.clawServoRight.setPosition(robot.CLAW_SERVO_RIGHT_UP);
        }

        if (gamepad1.a) {
            robot.clawServoLeft.setPosition(robot.CLAW_SERVO_LEFT_DOWN);
            robot.clawServoRight.setPosition(robot.CLAW_SERVO_RIGHT_DOWN);
        }

        if (gamepad1.b) {
            robot.clawServoLeft.setPosition(robot.CLAW_SERVO_LEFT_UP);
            robot.clawServoRight.setPosition(robot.CLAW_SERVO_RIGHT_UP);
            robot.boxGrabber.setPosition(robot.BOX_GRABBER_CLOSED);

            robot.boxMover.setTargetPosition(robot.BOX_MOVER_IN);
            robot.boxMover.setPower(robot.BOX_MOVER_SAFE_SPEED_FACTOR);

            robot.scissorLift.setTargetPosition(robot.SCISSOR_SAFE_POS);
            robot.scissorLift.setPower(robot.SCISSOR_SAFE_SPEED_FACTOR);


        }





//==========================================================
//						GamePad Two
//=========================================================
// =
//Change Collection Lift to joysticks/ add new gear ratio 4.5:1 with new motor never rest 60

       if (gamepad2.a) {
           robot.scissorLift.setTargetPosition(robot.SCISSOR_DOWN_POS);
           robot.indexLift = 0;
        }

       if (gamepad2.b) {
           if (!bPressed){
               robot.indexLift = robot.indexLift + 1;
               bPressed = true;
           }
           if (robot.indexLift > LIFT_MAX) {
               robot.indexLift = LIFT_MAX;
           }
           robot.scissorLift.setTargetPosition(robot.scissorposition[robot.indexLift]);
       }
       else
           bPressed = false;

        if (gamepad2.x) {
            if (!xPressed){
                robot.indexLift = robot.indexLift - 1;
                xPressed = true;
            }
            if (robot.indexLift < LIFT_MIN) {
                robot.indexLift = LIFT_MIN;
            }
            robot.scissorLift.setTargetPosition(robot.scissorposition[robot.indexLift]);
        }
        else
            xPressed = false;

        if (gamepad2.y) {
            robot.scissorLift.setTargetPosition(robot.SCISSOR_UP_POS);
        }



        //==========================================================
        //						Telemetry
        //==========================================================

        // this telemetry will report power setpoint to the Driver Station phone

        telemetry.addData("Left Trigger", "%.2f", gamepad1.left_trigger);

        telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);

        telemetry.addData("Scissor Lift Pos",  robot.scissorLift.getCurrentPosition());

        telemetry.addData("Pull Up Speed", "%.2f", pullupspeed);

        telemetry.addData("Left Bumper", gamepad1.left_bumper);

        telemetry.addData("Right Bumper", gamepad1.right_bumper);

        telemetry.addData("BRMotor", robot.rearRightDrive.getCurrentPosition());
        telemetry.addData("FRMotor", robot.frontRightDrive.getCurrentPosition());

        telemetry.addData("BLMotor", robot.rearLeftDrive.getCurrentPosition());
        telemetry.addData("FLMotor", robot.frontLeftDrive.getCurrentPosition());

        telemetry.addData("Box Mover",robot.boxMover.getCurrentPosition());
        telemetry.addData("Dpad Down",gamepad1.dpad_down);
        telemetry.addData("Dpad Up",gamepad1.dpad_up);
    }

    /*
     * Code will run ONCE after the driver hits STOP
     */

    @Override

    public void stop(){


        // Sets all motors to zero power

        robot.frontLeftDrive.setPower(0);
        robot.rearLeftDrive.setPower(0);

        robot.frontRightDrive.setPower(0);
        robot.rearRightDrive.setPower(0);
        //Ask and see if needed
        //robot.boxMover.setPower(0);
        //robot.scissorLift.setPower(0);


    }

} //end main