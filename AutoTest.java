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


/*
 * Autonomous Opmode GM3


@Autonomous(name="AutoTest", group="Wired")
@Disabled
public class AutoTest extends AutoLinearAbstract {

    // Declare OpMode members specific to this Autonomous Opmode variant.
    int colordetected = 0;

    @Override
    public void runOpMode() {

        // Execute the typical autonomous program elements.
        // super.runOpMode finishes as soon as the Drive Station start/play button is pressed.
        super.runOpMode();

        colordetected =0;


        //lowers lifts
        pullup_motor.goToAbsoluteDistance(5, .5);
        while (!pullup_motor.isMoveDone(LANDING_ERROR)) {
            telemetry.addLine("wait-landing");
            motorTelemetry(pullup_motor);
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                pullup_motor.stop();
                break;
            }
        }


        // Go straight to exit balancing stone
        driveTrain.goStraightToTarget(2, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight to exit balancing stone");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        pullup_motor.goToAbsoluteDistance(0, .5);
        //while (!pullup_motor.isMoveDone(LANDING_ERROR)) {
        //    telemetry.addLine("wait-landing");
        //}


        driveTrain.resetEncoders();

        driveTrain.goStraightToTarget(24, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight to exit balancing stone");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        while (generalTimer.seconds() > 3) {
            telemetry.addLine("Wait - Drive straight to elements");

            colorSensor1.evaluateColor();
            colorSensor2.evaluateColor();

            if (colorSensor1.colorGold || colorSensor2.colorGold) {  //gold
                colordetected = 2;
                driveTrain.stop();
                driveTrain.goStraightToTarget(18, DRIVE_TRAIN_DEFAULT_SPEED);

            }

            if (!colorSensor1.colorGold || !colorSensor2.colorGold) {
                colordetected = 1;
                driveTrain.stop();
                driveTrain.goStraightToTarget(-20, .5);
            }

            if (colorSensor1.colorGold || colorSensor2.colorGold || !colorSensor1.colorGold || !colorSensor2.colorGold)
                break;

            ColorTelemetry();
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - knock out gold or back up from white");

            if (colordetected == 1)
                telemetry.addLine("White detected");

            if (colordetected == 2)
                telemetry.addLine("Gold detected");

            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }

        if (colordetected ==1) {
            driveTrain.turnCcwToTarget(4, .5);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Wait - Drive train turn");
                driveTrainTelemetry();
                telemetry.update();
                if (autoTimer.seconds() > 28) {
                    driveTrain.stop();
                    break;
                }
            }

            driveTrain.goStraightToTarget(26, DRIVE_TRAIN_DEFAULT_SPEED);
            while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
                telemetry.addLine("Wait - Drive train move straight");
                driveTrainTelemetry();
                telemetry.update();
                if (autoTimer.seconds() > 28) {
                    driveTrain.stop();
                    break;
                }
            }
        }

        multi_End_servo.goToPosition(.8,0.02);
        generalTimer.reset();
        multi_Rotate_servo.goToPositionNow(.9);
        while (generalTimer.seconds()< 3) {
            telemetry.addLine("Servo dumps element ");

            if (autoTimer.seconds() > 28) {
                break;
            }
        }

        multi_Rotate_servo.goToPositionNow(.485);

        /*driveTrain.turnCcwToTarget(16, 0.5);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Turning the robot to the left elements");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 25) {
                driveTrain.stop();
                break;
            }
        }


        driveTrain.goStraightToTarget(72, DRIVE_TRAIN_DEFAULT_SPEED);
        while (!driveTrain.isMoveDone(MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES)) {
            telemetry.addLine("Wait - Drive train move straight to exit balancing stone");
            driveTrainTelemetry();
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }


        while (!pullup_motor.isMoveDone(LANDING_ERROR)) {
            telemetry.addLine("reset-lander");
            telemetry.update();
            if (autoTimer.seconds() > 28) {
                driveTrain.stop();
                break;
            }
        }



    }

}

*/