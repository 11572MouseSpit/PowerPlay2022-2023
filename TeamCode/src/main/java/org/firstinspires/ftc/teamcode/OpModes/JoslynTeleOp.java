package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "FieldCentricTeleOp", group = "Competition")

public class JoslynTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power=1;
        double rightX, rightY;
        double bumpCount = 0;
        boolean clawState = false;
        boolean clawReady = false;
        boolean toggleReadyUp = false;
        boolean toggleReadyDown = false;
        boolean fieldCentric = true;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        double liftPower = robot.LIFT_POWER_DOWN;
        ElapsedTime elapsedTime = new ElapsedTime();


        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);


        SampleMecanumDrive beans = new SampleMecanumDrive(hardwareMap);
        //sets motors to run without encoders
        beans.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = Math.toDegrees(robot.imu.getAbsoluteHeading()) - 90;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);


            beans.setWeightedDrivePower(
                    new Pose2d(
                            -(gamepad1.left_stick_y*Math.cos(Math.toRadians(theta))+gamepad1.left_stick_x*Math.sin(Math.toRadians(theta)))*1,
                            -(gamepad1.left_stick_x*Math.cos(Math.toRadians(theta))-gamepad1.left_stick_y*Math.sin(Math.toRadians(theta)))*1,
                            -gamepad1.right_stick_x*0.8

                    )
            );

            beans.update();


            GamepadEx gp1 = new GamepadEx(gamepad1);
            ButtonReader aReader = new ButtonReader(gp1, GamepadKeys.Button.A);
            ButtonReader bReader = new ButtonReader(gp1, GamepadKeys.Button.RIGHT_BUMPER);

            /*
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = -gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            robot.motorLF.setPower(Range.clip((v1), -power, power));
            robot.motorRF.setPower(Range.clip((v2), -power, power));
            robot.motorLR.setPower(Range.clip((v3), -power, power));
            robot.motorRR.setPower(Range.clip((v4), -power, power));

             */

            // Control which direction is forward and which is backward from the driver POV
 /*           if (gamepad1.y && (currentTime.time() - buttonPress) > 0.3) {
                if (theta2 == 180) {
                    theta2 = 0;
                } else {
                    theta2 = 180;
                }
                buttonPress = currentTime.time();
            }   // end if (gamepad1.x && ...)
*/

            /*  LIFT CONTROL  */
            if(gp1.isDown(GamepadKeys.Button.B)){
                bumpCount=0;
                robot.servoFinger.setPosition(robot.FINGER_IN);
                targetPosition= robot.LIFT_RESET;
            }
            //claw control
            if(aReader.isDown()&&clawReady){
                clawState=!clawState;
            }
            //forces claw to only open or close if button is pressed once, not held
            if(!aReader.isDown()){
                clawReady=true;
            }else{
                clawReady=false;
            }
            //apply value to claw
            if (clawState) {
                drive.openClaw();
            } else {
                drive.closeClaw();
            }

            if ((gamepad1.left_trigger > 0.1) || (gamepad2.left_trigger > 0.1)){
                targetPosition = targetPosition - 20;
                liftPower = robot.LIFT_POWER_UP;
            } else if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                targetPosition = targetPosition + 20;
                liftPower = robot.LIFT_POWER_DOWN;
            }

            /* Limit the range of the lift so as not to damage the robot */
            targetPosition = Range.clip(targetPosition, robot.LIFT_RESET, robot.LIFT_MAX_HEIGHT);

            drive.liftPosition(targetPosition, liftPower);

            /* Claw Control */
            if(!gp1.isDown(GamepadKeys.Button.RIGHT_BUMPER)){
                toggleReadyUp=true;
            }
            if(!gp1.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                toggleReadyDown=true;
            }

            //increase lift position
            if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)&&toggleReadyUp){

                toggleReadyUp=false;
                if(bumpCount<3){
                    bumpCount++;
                }

                //increase lift position
            }else if(gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)&&toggleReadyDown){

                toggleReadyDown=false;
                if(bumpCount>0){
                    bumpCount--;
                }
            }

            if (bumpCount == 0 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                targetPosition = robot.LIFT_RESET;
                robot.servoFinger.setPosition(robot.FINGER_IN);
            } else if (bumpCount == 1 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                targetPosition = robot.LIFT_LOW_JUNCTION;
            } else if (bumpCount == 2 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                targetPosition = robot.LIFT_MID_JUNCTION;
            } else if (bumpCount == 3 && (gamepad1.left_bumper || gamepad1.right_bumper)) {
                targetPosition = robot.LIFT_HIGH_JUNCTION;
            }

            if(bumpCount != 0) {
                robot.servoFinger.setPosition(robot.FINGER_OUT);
            }
            if(gamepad1.dpad_up) {
                robot.lamp.setPower(1);
            } else if (gamepad1.dpad_down){
                robot.lamp.setPower(0);
            }


            // Provide user feedback
 /*           telemetry.addData("V1 = ", v1);
            telemetry.addData("elapsed time = ", elapsedTime.time());
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);

  */
            telemetry.addData("Lift Position", bumpCount);
            telemetry.addData("Claw State", clawState);
            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
            telemetry.addData("Theta = ", theta);
            telemetry.addData("Theta2 = ", theta);
            telemetry.addData("IMU Value: ", theta);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class