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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


/**
 * Autonomous Opmode G2
 */

@Autonomous(name="Auto_Blue_Skystone_Side", group="Wired")
public class Auto_Blue_Skystone_Side extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.


    @Override
    public void runOpMode() {



        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();


        // Go straight to become parallel with blocks
        driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Move to align with blocks");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.StrafeRightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing right to blocks");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }



        //Scan for skystone and then move towards that


        driveTrain.goStraightToTarget(6, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving up next to the blocks");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }

        scissorLift.goToAbsoluteDistance(0, .5);
        while (!scissorLift.isMoveDone(5)){
            telemetry.addLine("Scissor Lift going down");
            motorTelemetryDegrees(scissorLift);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                scissorLift.stop();
                break;
            }
        }

        boxMover.goToAbsoluteDistance(BOX_MOVER_OUT, .7);
        while (!boxMover.isMoveDone(5)) {
            telemetry.addLine("Box mover going out");
            motorTelemetryDegrees(boxMover);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                boxMover.stop();
                break;
            }
        }



        scissorSides.goToPositionNow(SCISSOR_SIDES_IN);
        scissorTop.goToPositionNow(SCISSOR_TOP_DOWN);

        //Box mover in position
        //Go back wards with direction that the skystone was

        driveTrain.goStraightToTarget(-4, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving backwards out of the way");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.StrafeLeftToTarget(96, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Strafing left to foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }

        driveTrain.goStraightToTarget(4, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Moving up to the foundation");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }


        scissorTop.goToPositionNow(SCISSOR_TOP_UP);
        scissorSides.goToPositionNow(SCISSOR_SIDES_OUT);





    }
}




