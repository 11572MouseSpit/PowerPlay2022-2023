/*
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

import java.util.List;
import java.util.Objects;

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
@Autonomous(name = "Auto: Red Terminal", group = "Concept")

public class AutoRedCorner extends LinearOpMode {

    FtcDashboard dashboard;

    private static final String TFOD_MODEL_ASSET = "GenericSignalSleeve-Take1.tflite";
//    private static final String TFOD_MODEL_ASSET = "PP_Generic_SS.tflite";
    //    private static final String TFOD_MODEL_ASSET = "MouseSpit-Take1.tflite";
//    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";


    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };

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
    double position = 1;
    private final static HWProfile robot = new HWProfile();
    private LinearOpMode opMode = this;

    DriveClass drive = new DriveClass(robot, opMode);

    @Override

    public void runOpMode() {
        ElapsedTime elapsedTime = new ElapsedTime();
        double overshoot = 0;
        double turnError = 0;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /*
         * Setup the initial state of the robot
         */

        State autoState = State.DETECT_CONE;

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
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

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        while(!opModeIsActive() && !isStopRequested()) {
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
                        telemetry.addData("Park Position = ", position);
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
                        if(Objects.equals(recognition.getLabel(), "circle")){
                            position =1;
                        } else if(Objects.equals(recognition.getLabel(), "triangle")){
                            position = 2;
                        } else position = 3;

                        dashTelemetry.put("IMU Value :" , drive.getZAngle());
                        dashTelemetry.put("Motor Left Rear: ", robot.motorLeftRear.getCurrentPosition());
                        dashTelemetry.put("Motor Left Front: ", robot.motorLeftFront.getCurrentPosition());
                        dashTelemetry.put("Motor Right Rear: ", robot.motorRightRear.getCurrentPosition());
                        dashTelemetry.put("Motor Right Front: ", robot.motorRightFront.getCurrentPosition());
                        dashTelemetry.put("Motor Left Lift: ", robot.motorLeftLift.getCurrentPosition());
                        dashTelemetry.put("Motor Right Lift: ", robot.motorRightLift.getCurrentPosition());
                        dashTelemetry.put("# Objects Detected: ", updatedRecognitions.size());
                        dashTelemetry.put("Image             : ", recognition.getLabel());
                        dashTelemetry.put("Position          : ", position);
                        dashTelemetry.put("Confidence        : ", recognition.getConfidence() * 100 );
                        dashboard.sendTelemetryPacket(dashTelemetry);
                        telemetry.update();
                    }
                }
            }
        }  // end of while

        while(opModeIsActive()){

            switch (autoState) {
                case TEST:
                    drive.liftPosition(robot.LIFT_HIGH_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(3000);
                    //                    drive.driveDistance(.5, -90, 20);
                    autoState = State.HALT;
                    break;

                case DETECT_CONE:
                    telemetry.addData("PARK POSITION = ", position);
                    telemetry.update();
                    autoState = State.SCORE_LOW_JUNCTION;
                    break;

                case SCORE_LOW_JUNCTION:
                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(500);

                    // raise the lift to place the cone
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, robot.LIFT_POWER_UP);
                    drive.fingerExtend();

                    // drive forward to place the cone
                    drive.newDriveDistance(0.3, 0, 3);

                    // turn towards the low junction
                    turnError = drive.PIDRotate(43, 1);
                    while(Math.abs(turnError) > 1){
                        turnError = drive.PIDRotate(43, 1);
                    }

                    // drive forward to place the cone
                    // track overshoot drive distance so that a correction in the reverse direction can be made if necessary
                    overshoot = drive.newDriveDistance(0.3, 0, 2);

                    // release the cone
                    drive.fingerRetract();
                    sleep(75);
                    drive.openClaw();
                    sleep(150);

                    // drive back to head towards the cone stack
                    drive.newDriveDistance(0.3, 180, (2 + overshoot));

                    // point in the right direction
                    turnError = drive.PIDRotate(0, 1);
                    while(Math.abs(turnError) > 1){
                        turnError = drive.PIDRotate(0, 1);
                    }

                    // Lower the lift to place the cone
                    drive.resetLift(robot.LIFT_POWER_DOWN);
                    drive.fingerRetract();

                    autoState = State.CONE_5;
                    break;

                case CONE_5:
                    // push the signal cone out of the way
                    drive.newDriveDistance(0.6, 0, 55);
                    drive.newDriveDistance(0.3, 180, 2);

                    // raise the lift to collect a cone
                    drive.liftPosition(robot.LIFT_CONE_5, robot.LIFT_POWER_UP);

                    sleep(1000);

                    // back into position to correct for overshoot
                    drive.newDriveDistance(0.2, 180, overshoot);

                    // turn towards the stack of cones
                    turnError =  drive.PIDRotate(-90, 1);
                    while(Math.abs(turnError) > 1){
                        turnError = drive.PIDRotate(-90, 1);
                    }

                    // open the claw to grab the cone
                    drive.openClaw();

                    // drive to the stack of cones
                    drive.newDriveDistance(0.5, 0, 12);

                    // turn towards the stack of cones 2nd
                    turnError = drive.PIDRotate(-90, 2);
                    while(Math.abs(turnError) > 1){
                        turnError = drive.PIDRotate(-90, 1);
                    }

                    // drive to the stack of cones 2nd
                    drive.newDriveDistance(0.5, 0, 3);

                    // drive until it hits cone using the color sensor

                    elapsedTime.reset();

                    while(robot.sensorCone.getDistance(DistanceUnit.INCH) > 2 && elapsedTime.time() <= robot.WAIT_DRIVE_TO_CONE) {
                        drive.setDrivePower(robot.DRIVE_TO_CONE_POWER, robot.DRIVE_TO_CONE_POWER,
                                robot.DRIVE_TO_CONE_POWER, robot.DRIVE_TO_CONE_POWER);
                    }

                    //stop motors
                    drive.motorsHalt();

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(300);

                    // drive to the stack of cones
                    drive.newDriveDistance(0.5, 180, 1);

                    // lift the cone off the stack
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(200);
                    drive.fingerExtend();

                    // back away from the stack of cones
                    drive.newDriveDistance(0.7, 180, 17);

                    autoState = State.SCORE_LOW_JUNCTION2;
                    break;

                case SCORE_LOW_JUNCTION2:
                    // raise the lift to the low junction
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(200);
                    drive.fingerExtend();

                    // rotate to the 2nd low junction
                    drive.PIDRotate(-130, 1);
                    while(Math.abs(turnError) > 1){
                        turnError = drive.PIDRotate(-130, 1);
                    }

                    // drive towards the low junction to place the cone
                    drive.newDriveDistance(0.4, 0, 2.0);

                    // open the claw to release the cone
                    drive.fingerRetract();
                    sleep(50);
                    drive.openClaw();
                    sleep(300);


                    // back away from the junction
                    drive.newDriveDistance(0.5, 180, 2);

                    // lower the lift to collect the next cone
                    drive.liftPosition(robot.LIFT_CONE_4, robot.LIFT_POWER_DOWN);

                    //rotate towards the cone stack
                    drive.PIDRotate(-90, 2);

                    autoState = State.CONE_4;
                    break;

                case CONE_4:
                    // raise the lift to collect a cone
                    drive.liftPosition(robot.LIFT_CONE_4, robot.LIFT_POWER_UP);

                    // open the claw to grab the cone
                    drive.openClaw();

                    // drive to the stack of cones
                    drive.newDriveDistance(0.5, 0, 20);

                    elapsedTime.reset();

                    // drive until it hits cone
                    while(robot.sensorCone.getDistance(DistanceUnit.INCH) > 2 && elapsedTime.time() <= robot.WAIT_DRIVE_TO_CONE) {
                        drive.setDrivePower(robot.DRIVE_TO_CONE_POWER, robot.DRIVE_TO_CONE_POWER,
                                robot.DRIVE_TO_CONE_POWER, robot.DRIVE_TO_CONE_POWER);
                    }

                    //stop motors
                    drive.motorsHalt();

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(300);

                    // drive to the stack of cones
                    drive.newDriveDistance(0.3, 180, 1.5);

                    // lift the cone off the stack
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(250);
                    drive.fingerExtend();
                    sleep(250);

                    // back away from the stack of cones
                    drive.newDriveDistance(0.5, 180, 17);
                    autoState = State.SCORE_HIGH_JUNCTION;
                    break;

                case SCORE_HIGH_JUNCTION:
                    // rotate towards the high junction
                    drive.PIDRotate(42.5, 2);

                    // raise the lift to the high junction
                    drive.liftPosition(robot.LIFT_HIGH_JUNCTION, robot.LIFT_POWER_UP);
                    drive.fingerExtend();
                    sleep(400);

                    // drive towards the low junction to place the cone
                    drive.newDriveDistance(0.3, 180, 0.75);

                    sleep(1000);

                    // lower the lift to place the cone
                    drive.fingerRetract();
                    sleep(100);

                    // open the claw to release the cone
                    drive.openClaw();

                    // back away from the junction
                    drive.newDriveDistance(0.3, 0, .75);
                    sleep(300);

                    // rotate back towards the cone stack
                    drive.PIDRotate(-90,2);
                    sleep(300);
                    drive.resetLift(robot.LIFT_POWER_DOWN);

                    autoState = State.PARK;
                    break;

                case CONE_3:
                    //Todo: Test this section of code

                    // drive forward to pick up another cone
                    drive.newDriveDistance(0.5, 0, 10);

                    // correct heading if necessary
                    drive.PIDRotate(-90,2);


                    // Set the lift to the right heigth to grab the next cone
                    drive.liftPosition(robot.LIFT_CONE_3, robot.LIFT_POWER_UP);
                    drive.fingerExtend();

                    // drive forward to pick up another cone
                    drive.newDriveDistance(0.5, 0, 6);

                    elapsedTime.reset();

                    // drive until it hits cone
                    while(robot.sensorCone.getDistance(DistanceUnit.INCH) > 2 && elapsedTime.time() <= robot.WAIT_DRIVE_TO_CONE) {
                        drive.setDrivePower(robot.DRIVE_TO_CONE_POWER, robot.DRIVE_TO_CONE_POWER,
                                robot.DRIVE_TO_CONE_POWER, robot.DRIVE_TO_CONE_POWER);
                    }

                    //stop motors
                    drive.motorsHalt();

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(1000);

                    //back away from the stack slightly
                    drive.newDriveDistance(0.4, 180, 1);

                    // lift the cone off the stack
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(300);
                    drive.fingerExtend();

                    // back away from the stack of cones
                    drive.newDriveDistance(0.5, 180, 24);

                    autoState = State.SCORE_MID_JUNCTION;
                    break;

                case SCORE_MID_JUNCTION:
                    //Todo: Test this section of code


                    // rotate towards the mid junction
                    drive.PIDRotate(135, 2);

                    // raise the lift to the mid junction
                    drive.liftPosition(robot.LIFT_MID_JUNCTION, robot.LIFT_POWER_UP);
                    drive.fingerExtend();
                    sleep(500); // allow the robot to reach scoring position

                    // drive towards the mid junction to place the cone
                    drive.newDriveDistance(0.3, 0, 0);

                    // lower the lift to place the cone
                    drive.fingerRetract();
                    sleep(100);
                    drive.resetLift(robot.LIFT_POWER_DOWN);
                    sleep(500);

                    // open the claw to release the cone
                    drive.openClaw();

                    // back away from the junction
                    drive.newDriveDistance(0.3, 180, 0);

                    // rotate back towards the outside wall to pick up another cone
                    drive.PIDRotate(-90, 2);

                    autoState = State.CONE_2;
                    break;


                case CONE_2:
                    //Todo: Test this section of code

                    // drive forward to pick up another cone
                    drive.newDriveDistance(0.5, 0, 10);

                    // correct heading if necessary
                    drive.PIDRotate(-90,2);

                    // Set the lift to the right heigth to grab the next cone
                    drive.liftPosition(robot.LIFT_CONE_2, 0.9);
                    drive.fingerExtend();

                    // drive forward to pick up another cone
                    drive.newDriveDistance(0.5, 0, 10);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(1000);

                    //back away from the stack slightly
                    drive.newDriveDistance(0.4, 180, 1);

                    // lift the cone off the stack
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, 0.9);
                    drive.fingerExtend();
                    sleep(500);

                    // back away from the stack of cones
                    drive.newDriveDistance(0.5, 180, 48);

                    autoState = State.SCORE_HIGH_JUNCTION2;
                    break;

                case SCORE_HIGH_JUNCTION2:
                    //Todo: Test this section of code

                    // rotate towards the high junction
                    drive.PIDRotate(-135, 2);

                    // raise the lift to the mid junction
                    drive.liftPosition(robot.LIFT_HIGH_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(300); // allow the robot to reach scoring position
                    drive.fingerExtend();

                    // drive towards the low junction to place the cone
                    drive.newDriveDistance(0.3, 0, 4);

                    // lower the lift to place the cone
                    drive.fingerRetract();
                    sleep(100);
                    drive.resetLift(robot.LIFT_POWER_DOWN);
                    sleep(500);

                    // open the claw to release the cone
                    drive.openClaw();

                    // back away from the junction
                    drive.newDriveDistance(0.3, 180, 0);

                    // rotate back towards the outside wall to pick up another cone
                    drive.PIDRotate(-90, 2);

                    autoState = State.PARK;
                    break;


                case PARK:

                    if(position == 1) {
                        // drive forward to park position 1
                        drive.newDriveDistance(0.3, 0,28);

                    } else if (position == 2) {
                        // return to starting position
                        drive.newDriveDistance(0.3, 180,2);

                    } else {
                        // drive to park position 3
                        drive.newDriveDistance(0.45, 180, 28);
                    }

                    autoState = State.HALT;

                    break;

                case HALT:

                    // Stop all motors
                    drive.newMotorsHalt();

                    // End the program
                    requestOpModeStop();

                    break;
            }   // end of the switch state


        } // end of while(opModeIsActive())
        // End the program
        requestOpModeStop();

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
        TEST, DETECT_CONE, SCORE_LOW_JUNCTION, SCORE_LOW_JUNCTION2, CONE_5, CONE_4,
        CONE_3, CONE_2, CONE_1, SCORE_MID_JUNCTION, SCORE_HIGH_JUNCTION, SCORE_HIGH_JUNCTION2,
        PARK, HALT
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