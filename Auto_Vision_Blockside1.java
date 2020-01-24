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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * Autonomous Opmode GM5
 */

@Autonomous(name="Auto_Vision_FoundationSide", group="Wired")
//@Disabled
public class Auto_Vision_Blockside1 extends AutoLinearVU3 {
    int blockPosition;
    // Declare OpMode members specific to this Autonomous Opmode variant.

    @Override
    public void runOpMode() {


        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();
        blockPosition = 1;

        driveTrain.StrafeRightToTarget(38, .5);
        scissorLift.goToAbsoluteDistance(SCISSOR_LIFT_PICK_POS,0.5);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Strafing right to stones");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


        if (RobotEyes(allTrackables) != 1) {
            blockPosition = 2;
            driveTrain.StrafeRightToTarget(7, .6);
        }

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }
        if (blockPosition == 2)
            driveTrain.StrafeLeftToTarget(7, .6);

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        if (blockPosition == 2)
            driveTrain.goStraightToTarget(-8, .6);

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }
        //- done with 2


        if (RobotEyes(allTrackables) != 1) {
            blockPosition = 3;
            driveTrain.StrafeRightToTarget(7, .6);
        }

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }
        if (blockPosition == 3)
            driveTrain.StrafeLeftToTarget(7, .6);

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        if (blockPosition == 3)
            driveTrain.goStraightToTarget(-8, .6);

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }
        // - done with 3
        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        boxMover.goToAbsoluteDistance(BOX_MOVER_PICK_POS_OUT,0.5);
        while (!boxMover.isMoveDone(BOX_MOVER_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        boxGrabber.goToPosition(BOX_GRABBER_CLOSED,.003);

        scissorLift.goToRelativeDistance(100,.5);
        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.StrafeLeftToTarget(30,.5);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        if (blockPosition==1)
            driveTrain.goStraightToTarget(40,.5);

        if (blockPosition ==2)
            driveTrain.goStraightToTarget(48,.5);

        if (blockPosition == 3)
            driveTrain.goStraightToTarget(56,.5);

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        driveTrain.StrafeRightToTarget(24,.5);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        boxGrabber.goToPositionNow(BOX_GRABBER_OPEN);
        boxMover.goToAbsoluteDistance(0,0.5);

        clawServoLeft.goToPositionNow(CLAW_SERVO_LEFT_DOWN);
        clawServoRight.goToPosition(CLAW_SERVO_RIGHT_DOWN,0.003);

        driveTrain.StrafeLeftToTarget(32,.5);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }

        }
        scissorLift.goToAbsoluteDistance(0.0,0.5);

        clawServoRight.goToPositionNow(CLAW_SERVO_LEFT_UP);
        clawServoLeft.goToPosition(CLAW_SERVO_LEFT_UP,.003);

        driveTrain.goStraightToTarget(-24,.5);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }

        while (!scissorLift.isMoveDone(SCISSOR_LIFT_POSITION_ERROR)) {
            telemetry.addLine("XXX");
            driveTrainTelemetry();
            telemetry.update();
            if (Kill(28)) {
                break;
            }
        }


    }
}





