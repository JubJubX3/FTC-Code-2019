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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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
public abstract class AutoLinearVU3 extends LinearOpMode {

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
            clawServoLeft,
            boxGrabber,
            clawServoRight;



    ElapsedTime
            generalTimer = new ElapsedTime(), // General/multipurpose timer
            autoTimer = new ElapsedTime();    // Autonomous timer


    // CONSTANTS
    //static final int
    public List<VuforiaTrackable> allTrackables;


    // CONSTANTS
    //static final int
    double
            XPosition,
            YPosition,
            ZPosition;


    double
            roll,
            pitch,
            heading;


    String targetFound;

    final static double
            MAX_DRIVE_TRAIN_POSITION_ERROR_INCHES = 0.25,

    DRIVE_TRAIN_PIVOT_SPEED = 0.2,
            DRIVE_TRAIN_DEFAULT_SPEED = 0.5,
            DRIVE_TRAIN_STRAIGHT_SPEED = 0.6,

    BOX_GRABBER_CLOSED = 0.52,
            BOX_GRABBER_OPEN = 0.7,
            CLAW_SERVO_LEFT_UP = 0.65,
            CLAW_SERVO_LEFT_DOWN = 0.05,
            CLAW_SERVO_RIGHT_UP = 0.05,
            CLAW_SERVO_RIGHT_DOWN = 0.65;


    final static boolean
            FORWARD = false,
            REVERSE = true;

    final static int
            SCISSOR_LIFT_PICK_POS = 3500,
            SCISSOR_LIFT_POSITION_ERROR = 100,
            BOX_MOVER_PICK_POS_OUT = 3000,
            BOX_MOVER_POSITION_ERROR = 100;



    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " AeeN/3r/////AAABmf1me9Nb2Ej/q0VZKiFuQ/EiiMAn/BJDOd5n6hfxmP6/m7RdZX0DhKBijOtTZ9MtmLZDxr2s0/kjmIu/3n95SpT3t1p4qoZXEUyutIhiI5ZrqBw7yo33KDHtmzQ69S80OwSbyg2dlSnjX8wvA4JUVda/Xuxx2MtbaIdYKhSEhZr3hq0DOb4MGo9NAITEZac/5PkfYGm4QuPxtz7wFsTtnZO0AoR6nmBgJx2W7iUzYYv4FuWuthZLTl5T//LkCNhznDzU8DEZP3amALFY2f0h4MwpCmGqRCEtfUc3SRzmdMbWhX1Hm7fEraahAhGchdivrhnq2bNMw2PPYm2YTVCoSNFR0vFRE3YmKihMLWiio6Zu ";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    // WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

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


        /* Color sensor constructor: hardwareMap, sensor name, sensor I2C address */
        // colorLeftJewel = new DeviceColorSensor(hardwareMap,"left_jewel_color",0x3c);
        //   colorRightJewel = new DeviceColorSensor(hardwareMap,"right_jewel_color",0x30);

        /* Target-Servo constructor: hardwareMap, servo name, initial servo position */
        boxGrabber = new DeviceTargetServo(hardwareMap,"box_grabber",BOX_GRABBER_OPEN);
        clawServoLeft = new DeviceTargetServo(hardwareMap, "claw_servo_left", CLAW_SERVO_LEFT_UP);
        clawServoRight = new DeviceTargetServo(hardwareMap, "claw_servo_right",CLAW_SERVO_RIGHT_UP);
        /* INITIALIZE ROBOT - INITIALIZE ROBOT OBJECTS AND CLASSES*/







        driveTrain = new mecanumDrive(hardwareMap,"front_left_drive",REVERSE,"front_right_drive",FORWARD,"rear_left_drive",REVERSE,"rear_right_drive",FORWARD,1120,1.0,2.0);

        /* Target-Motor constructor: hardwareMap, motor name, motor direction,
                              encoder counts per output shaft revolution, gear ratio, wheel radius */
        scissorLift = new DeviceTargetMotor(hardwareMap,"scissor_lift",REVERSE,1680);

        boxMover = new DeviceTargetMotor(hardwareMap,"box_mover",REVERSE,1440);


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

        /*
         * Retrieve the camera we are to use.
         */
        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        //parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");


        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }



        //Hey where's your family from out of america
        // sophis wanna know it
        //greece cool thx




        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        targetsSkyStone.activate();
        // Report initialization complete
        telemetry.addLine("Initialization Complete");
        telemetry.addLine("Hold for Start");
        telemetry.update();


        // WAIT FOR THE GAME TO START (driver presses PLAY)


        autoTimer.reset();  // Reset/restart the autotimer



        // GAME STARTED - BEGIN AUTONOMOUS OPERATIONS

    }

    public int RobotEyes (List<VuforiaTrackable> patterns) {
        int indexTarget ;
        indexTarget=0;
        targetVisible = false;

        for (VuforiaTrackable trackable : patterns) {
            indexTarget = indexTarget +1;
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());

                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            XPosition = translation.get(0)/mmPerInch;
            YPosition = translation.get(1)/mmPerInch;
            ZPosition = translation.get(2)/mmPerInch;

            // express the rotation of the robot in degrees.

            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            roll = rotation.firstAngle;
            pitch = rotation.secondAngle;
            heading = rotation.thirdAngle;

            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }


        else {
            telemetry.addData("Visible Target", "none");

        }

        if(!targetVisible)
            indexTarget = 0;

        telemetry.addData("index"," = ", indexTarget);
        telemetry.update();

        return indexTarget;
    }


    /* -------------------------------------------------------
     * Method: driveTrainTelemetry
     * Purpose: Report the position and speed of the drive train wheels
     * ------------------------------------------------------- */
    void driveTrainTelemetry () {
        telemetry.addLine("Left Drive Front Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.front.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.front.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.front.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.front.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.front.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.front.leftSpeed);


        telemetry.addLine("Left Drive Rear Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.rear.motorLeft.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.rear.motorLeft.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.rear.motorLeft.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.rear.motorLeft.targetCount);
        telemetry.addData("  Is Busy",driveTrain.rear.motorLeft.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.rear.leftSpeed);

        telemetry.addLine("Right Drive Front Motor");
        telemetry.addData("  Position in EngUnits","%.2f", driveTrain.front.motorRight.getPosition());
        telemetry.addData("  Target in EngUnits","%.2f", driveTrain.front.motorRight.targetPosition);
        telemetry.addData("  Position in Counts",driveTrain.front.motorRight.targetMotor.getCurrentPosition());
        telemetry.addData("  Target in Counts",driveTrain.front.motorRight.targetCount);
        telemetry.addData("  Is Busy",driveTrain.front.motorRight.targetMotor.isBusy());
        telemetry.addData("  Speed",driveTrain.front. rightSpeed);

        telemetry.addLine("Right Drive Rear Motor");
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
        if( autoTimer.seconds()>= autoTime) {

            driveTrain.stop();
            scissorLift.stop();

            eStop = true;

        }
        else
            eStop = false;

        return eStop;



    }

}







