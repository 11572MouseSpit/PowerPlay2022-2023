/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

import java.util.List;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Auto: Blue Terminal", group = "Concept")

public class AutoBlueCorner extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

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
            "AfHl2GP/////AAABmeJc93xOhk1MvZeKbP5E43taYJ6kodzkhsk5wOLGwZI3wxf7v1iTx2Mem/VZSEtpxb3U2fMO7n0EUxSeHRWhOXeX16dMFcjfalezjo3ZkzBuG/y2r4kgLwKs4APyAIClBAon+tf/W/4NkTkYuHGo8zZ0slH/iBpqxvblpNURsG5h4VxPFgF5D/FIfmjnddzQpa4cGarle/Zvuah6q2orUswun31P6ZLuIJvdOIQf7o/ruoRygsSXfVYc35w+Xwm+bwjpZUNzHHYvRNrp0HNWC3Fr2hd0TqWKIIYlCoHj0m5OKX22Ris23V8PdKM/i4/ZIy8JewJXetv1rERC5bfHmUXCS4Rl7RjR+ZscQ5aA0nr8";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    double position = 3;
    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    DriveClass drive = new DriveClass(robot, opMode);

    @Override

    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /*
         * Setup the initial state of the robot
         */

        State autoState = State.DETECT_CONE;

        robot.init(hardwareMap);


        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        while(!opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        if(recognition.getLabel() == "1 Bolt"){
                            position =1;
                        } else if(recognition.getLabel() == "2 Bulb" ){
                            position = 2;
                        } else position = 3;
                    }
                    telemetry.update();
                }
            }

        }  // end of while

        waitForStart();

        while(opModeIsActive()){

            switch (autoState) {
                case TEST:
                    drive.liftHigh();
                    sleep(3000);
                    //                    drive.driveDistance(.5, -90, 20);
                    autoState = AutoBlueCorner.State.HALT;

                    break;

                case DETECT_CONE:
                    autoState = AutoBlueCorner.State.SCORE_LOW_JUNCTION;
                    break;

                case SCORE_LOW_JUNCTION:
                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(500);
                    // strafe over to scoring position
                    drive.newDriveDistance(0.5, -90, 11);

                    // raise the lift to place the cone
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION);
                    sleep(1000);

                    // drive forward to place the cone
                    drive.newDriveDistance(0.5, 0, 4);

                    // Lower the lift to place the cone
                    drive.resetLift();
                    sleep(1000);
                    drive.openClaw();

                    // back away from the junction
                    drive.newDriveDistance(0.5, 180, 4);

                    //strafe back to starting position
                    drive.newDriveDistance(0.5, 90, 12);

                    // realign towards the signal cone
                    drive.newPIDRotate(0);
                    autoState = AutoBlueCorner.State.FIRST_CONE_STACK;

                    break;

                case FIRST_CONE_STACK:
                    // push the signal cone out of the way
                    drive.newDriveDistance(0.5, 0, 67);

                    // back into position to pick up the second cone
                    drive.newDriveDistance(0.5, 180, 8);

                    // turn towards the stack of cones
                    drive.newPIDRotate(90);

                    // raise the lift to collect a cone
                    drive.liftPosition(155);

                    // open the claw to grab the cone
                    drive.openClaw();

                    // drive to the stack of cones
                    drive.newDriveDistance(0.5, 0, 12);

                    // turn towards the stack of cones
                    drive.newPIDRotate(90);

                    // drive to the stack of cones
                    drive.newDriveDistance(0.5, 0, 12);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(1000);

                    // lift the cone off the stack
                    drive.liftPosition(420);
                    sleep(500);

                    // back away from the stack of cones
                    drive.newDriveDistance(0.5, 180, 26);

                    autoState = AutoBlueCorner.State.SCORE_LOW_JUNCTION2;
                    break;

                case SCORE_LOW_JUNCTION2:
                    // strafe towards the 2nd low junction
                    drive.newDriveDistance(0.5, 90, 14);

                    // raise the lift to the low junction
                    drive.liftLow();
                    sleep(500);

                    // drive towards the low junction to place the cone
                    drive.newDriveDistance(0.3, 0, 3);

                    // lower the lift to place the cone
                    drive.resetLift();
                    sleep(300);

                    // open the claw to release the cone
                    drive.openClaw();

                    // back away from the junction
                    drive.newDriveDistance(0.3, 180, 4);

                    // strafe back into position to pick up another cone
                    drive.newDriveDistance(0.5, -90, 12);

                    autoState = AutoBlueCorner.State.PARK;
                    break;

                case SECOND_CONE_STACK:
                    // raise the lift to collect a cone
                    drive.liftPosition(100);

                    // open the claw to grab the cone
                    drive.openClaw();

                    // drive to the stack of cones
                    drive.newDriveDistance(0.5, 0, 25);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(1000);

                    // lift the cone off the stack
                    drive.liftPosition(150);
                    sleep(500);

                    // back away from the stack of cones
                    drive.newDriveDistance(0.5, 180, 50);
                    autoState = AutoBlueCorner.State.SCORE_HIGH_JUNCTION;
                    break;

                case SCORE_CORNER:
//                    drive.driveDistance(0.25, 180, 2);
                    drive.driveByTime(0.25, -90,.75);
                    //  drive.PIDRotate(0,2);
                    //  drive.PIDRotate(0,2);
                    // drive.robotCorrect2(0.25, 0,.25);


                    autoState = AutoBlueCorner.State.HALT;
                    break;

                case SCORE_HIGH_JUNCTION:
                    // strafe towards the 2nd low junction
                    drive.newDriveDistance(0.5, -90, 14);

                    // raise the lift to the low junction
                    drive.liftHigh();
                    sleep(500);

                    // drive towards the low junction to place the cone
                    drive.newDriveDistance(0.3, 0, 0);

                    // lower the lift to place the cone
                    drive.resetLift();
                    sleep(700);

                    // open the claw to release the cone
                    drive.openClaw();

                    // back away from the junction
                    drive.newDriveDistance(0.3, 180, 0);

                    // strafe back into position to pick up another cone
                    drive.newDriveDistance(0.5, 90, 12);

                    autoState = AutoBlueCorner.State.PARK;
                    break;
                case PARK:

                    if(position == 1) {
                        // drive forward to park position 1
                        drive.newDriveDistance(0.3, 0,28);

                    } else if (position == 2) {
                        // return to starting position
                        drive.newDriveDistance(0.3, 0,3);

                    } else {
                        // drive to park position 3
                        drive.newDriveDistance(0.3, 180, 21);
                    }

                    autoState = AutoBlueCorner.State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    drive.newMotorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state


        } // end of while(opModeIsActive())

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    enum State {
        TEST, DETECT_CONE, SCORE_LOW_JUNCTION, SCORE_LOW_JUNCTION2, FIRST_CONE_STACK, SCORE_CORNER, SECOND_CONE_STACK, SCORE_HIGH_JUNCTION, PARK, HALT;
    }   // end of enum State

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}