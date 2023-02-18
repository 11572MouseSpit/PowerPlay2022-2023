/*
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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
@Autonomous(name = "Auto: RR Red Terminal", group = "Concept")

public class AutoRedCornerRR extends LinearOpMode {

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
    SampleMecanumDrive rrDrive = null;
    DriveClass drive = new DriveClass(robot, opMode);
    TrajectorySequence trajectory = null;
    private double xOffset = -36;
    private double yOffset = -63;
    private Pose2d currentPose = new Pose2d(xOffset, yOffset, Math.toRadians(90));

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
        rrDrive = new SampleMecanumDrive(hardwareMap);
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
            rrDrive.update();

            switch (autoState) {
                case TEST:
                    drive.liftPosition(robot.LIFT_HIGH_JUNCTION, robot.LIFT_POWER_UP);
                    sleep(3000);
                    //                    drive.driveDistance(.5, -90, 20);
                    autoState = State.LOW_TEST_SPLINE;
                    break;

                case DETECT_CONE:
                    telemetry.addData("PARK POSITION = ", position);
                    telemetry.update();

                    // set pose estimate
                    rrDrive.setPoseEstimate(currentPose);

                    autoState = State.JUST_MOVE_TEST;
                    break;

                //todo: DONT USE!
                case JUST_MOVE_TEST:
                    drive.closeClaw();
                    sleep(200);
                    //LOW JUNCTION 1
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            // temp wait for recording, remove later
                            // .waitSeconds(10)

                            .addDisplacementMarker(() -> {
                                drive.liftPosition(robot.LIFT_LOW_JUNCTION, 0.5);
                                drive.fingerExtend();
                            })
//                            .forward(5)
//                            .turn(Math.toRadians(-45))
//                            .forward(1)
                            .lineToLinearHeading(new Pose2d(0 + xOffset, 24 + yOffset, Math.toRadians(-45)))
                            .back(1)
                            .build();

                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    sleep(300);
                    //drop cone
                    drive.openClaw();
                    drive.fingerRetract();
                    sleep(400);
                    drive.liftPosition(robot.LIFT_CONE_5, 0.3);

                    //CONE 5
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            // drop cone marker
                            .back(2)
//                            .turn(Math.toRadians(45))
                            // must always add x and y offsets!
                            .lineToLinearHeading(new Pose2d(-2 + xOffset, 48 + yOffset, Math.toRadians(180)))

                            // go for cone 5

                            .forward(23)
                            .addDisplacementMarker(() -> {
                                drive.openClaw();
                                sleep(200);
                                drive.liftPosition(robot.LIFT_CONE_5, 0.3);
                                sleep(300);
                            })
                            .build();

                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    //GET CONE
                    drive.closeClaw();
                    sleep(400);

                    //HIGH JUNCTION
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            //score high junction

                            //.back(25)
                            //.turn(Math.toRadians(-225))
                            .back(0.25)
                            .addDisplacementMarker(() -> {
                                drive.liftPosition(robot.LIFT_LOW_JUNCTION, 0.3);
                            })
                            .waitSeconds(0.25)
                            .back(4)

                            .lineToLinearHeading(new Pose2d(-4 + xOffset, 50 + yOffset, Math.toRadians(51)))
                            .back(1.5)
                            // correct error
                            //.lineTo(new Vector2d(-2 + xOffset, 50 + yOffset))
//                            .forward(6)
                            // drop cone
                            .build();

                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    //DROP CONE
                    sleep(50);
                    drive.fingerExtend();
                    drive.liftPosition(robot.LIFT_HIGH_JUNCTION, 0.5);
                    sleep(1000);
                    drive.openClaw();
                    drive.fingerRetract();
                    sleep(600);
                    drive.liftPosition(robot.LIFT_CONE_4, 0.3);
                    sleep(300);

                    //CONE 4
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
//                            .back(6)

                            //go for cone 4

                            // .turn(Math.toRadians(135))
                            // .forward(25)
//                            .lineToLinearHeading(new Pose2d(-2 + xOffset, 50 + yOffset, Math.toRadians(180)))
                            .lineTo(new Vector2d(-3.75 + xOffset, 50 + yOffset)) // robot at -5, 50 (needs to be corrected)
                            .turn(Math.toRadians(135))
                            //correct
                            .forward(26)
                            .build();

                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    // close claw
                    drive.closeClaw();
//                    drive.fingerRetract();
                    sleep(400);

                    //MID JUNCTION
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            // score mid junction

//                            .back(0.25)
                            .forward(0.5)

                            .addDisplacementMarker(() -> {
                                drive.liftPosition(robot.LIFT_MID_JUNCTION, 0.3);
                                drive.fingerExtend();
                            })

                            .waitSeconds(0.25)
                            .back(4)

                            .lineToLinearHeading(new Pose2d(-4.5 + xOffset, 51 + yOffset, Math.toRadians(-47)))
                            .back(2.75)
                            //correct
                            //.lineTo(new Vector2d(-2 + xOffset, 50 + yOffset))
//                            .forward(6)
                            .build();
                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    //drop cone
                    drive.openClaw();
                    drive.fingerRetract();
                    sleep(300);
                    drive.liftPosition(robot.LIFT_CONE_3, 0.3);

                    //CONE 3
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)

                            .lineToLinearHeading(new Pose2d(-2 + xOffset, 50 + yOffset, Math.toRadians(-45)))
                            .turn(Math.toRadians(225))
                            .forward(26)
                            // grab cone
                            .build();
//                    rrDrive.followTrajectorySequence(trajectory);
//                    currentPose = trajectory.end();

                    //
                    drive.closeClaw();

                    // PARK
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            // back up and raise lift
                            .back(2)
                            .addDisplacementMarker(() -> {
                                drive.liftPosition(robot.LIFT_LOW_JUNCTION, 0.3);
                            })
                            .back(3)

                            //turn to parking position
                            .lineToLinearHeading(new Pose2d(-2 + xOffset, 51 + yOffset, Math.toRadians(0)))

                            // leave at the end of the program
                            .build();


                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();
                    drive.openClaw();

                    autoState = State.PARK;
                    break;


                case SCORE_LOW_JUNCTION:
                    // set pose estimate
                    rrDrive.setPoseEstimate(currentPose);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(100);
                    // turn to pole

                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            .forward(5)
                            .turn(Math.toRadians(-45))
                            .build();
                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    // lift arm then drive forward and drop
                    drive.fingerExtend();
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION, 0.4);
                    sleep(400);
                    Trajectory traj1 = rrDrive.trajectoryBuilder(currentPose)
                            .forward(3)
                            .build();
                    rrDrive.followTrajectory(traj1);
                    currentPose = traj1.end();

                    //wait then open
                    sleep(300);
                    drive.openClaw();
                    drive.fingerRetract();

                    //wait lower then close
                    sleep(250);
                    drive.liftPosition(robot.LIFT_RESET, robot.LIFT_POWER_DOWN);
                    sleep(400);

                    autoState = State.CONE_5;
                    break;
                case CONE_5:
                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            .back(3)
                            .turn(Math.toRadians(0))
                            .waitSeconds(1)
                            .build();
                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();
                    rrDrive.waitForIdle();

                    trajectory = rrDrive.trajectorySequenceBuilder(currentPose)
                            .splineTo(new Vector2d(0 + xOffset, 30 + yOffset), Math.toRadians(180))
                            .build();
                    rrDrive.followTrajectorySequence(trajectory);
                    currentPose = trajectory.end();

                    autoState = State.HALT;
                    break;
                case PARK:

                    if(position == 1) {
                        // drive forward to park position 1
                        drive.newDriveDistance(0.3, 180,28);

                    } else if (position == 2) {
                        // return to starting position
                        drive.newDriveDistance(0.3, 180,5);

                    } else {
                        // drive to park position 3
                        drive.newDriveDistance(0.45, 0, 28);
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
        PARK, LOW_TEST_SPLINE, JUST_MOVE_TEST, HALT
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