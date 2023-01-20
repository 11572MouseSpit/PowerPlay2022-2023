package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@TeleOp(name = "Teleop Mode", group = "Competition")

public class MecanumTeleOp extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode(){
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power=1;
        double rightX, rightY;
        double TargetRotation = 0;
        double OldRotation = 0;
        boolean rotateEnabled = false;
        boolean fieldCentric = false;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        double liftPower = robot.LIFT_POWER_DOWN;
        ElapsedTime elapsedTime = new ElapsedTime();
        double RFrotatePower = robot.TURN_SPEED;
        double LFrotatePower = -robot.TURN_SPEED;
        double LRrotatePower = -robot.TURN_SPEED;
        double RRrotatePower = robot.TURN_SPEED;


        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() + 90;
//                        robot.imu.getAngularOrientation().firstAngle + 90;
            } else {
                theta = 0;      // do not adjust for the angular position of the robot
            }

            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = -gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta + theta2)) - rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta + theta2)) + rightX + rightY);

            if(robot.imu.getAbsoluteHeading() - OldRotation >= TargetRotation && rotateEnabled) {
                rotateEnabled = false;
                TargetRotation = 0;
                OldRotation = 0;
            }

            if(!rotateEnabled) {
                robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -power, power));
                robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -power, power));
                robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -power, power));
                robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -power, power));
            } else {
                robot.motorLF.setPower(LFrotatePower);
                robot.motorRF.setPower(RFrotatePower);
                robot.motorLR.setPower(LRrotatePower);
                robot.motorRR.setPower(RRrotatePower);
            }

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
            if(gamepad1.a || gamepad2.a){
                targetPosition = robot.LIFT_RESET;
                robot.servoFinger.setPosition(robot.FINGER_IN);
                liftPower = robot.LIFT_POWER_DOWN;
            } else if(gamepad1.b || gamepad2.b){
                targetPosition = robot.LIFT_LOW_JUNCTION;
                robot.servoFinger.setPosition(robot.FINGER_OUT);
                liftPower = robot.LIFT_POWER_UP;
            } else if(gamepad1.x || gamepad2.x) {
                targetPosition = robot.LIFT_MID_JUNCTION;
                robot.servoFinger.setPosition(robot.FINGER_OUT);
                liftPower = robot.LIFT_POWER_UP;
            } else if(gamepad1.y || gamepad2.y) {
                targetPosition = robot.LIFT_HIGH_JUNCTION;
                robot.servoFinger.setPosition(robot.FINGER_OUT);
                liftPower = robot.LIFT_POWER_UP;
            }

            if ((gamepad1.left_trigger > 0.1) || (gamepad2.left_trigger > 0.1)){
                targetPosition = targetPosition + 20;
                liftPower = robot.LIFT_POWER_UP;
            } else if ((gamepad1.right_trigger > 0.1) || (gamepad2.right_trigger > 0.1)) {
                targetPosition = targetPosition - 20;
                liftPower = robot.LIFT_POWER_DOWN;
            }

            /* Limit the range of the lift so as not to damage the robot */
            targetPosition = Range.clip(targetPosition, robot.LIFT_RESET, robot.LIFT_MAX_HEIGHT);

            drive.liftPosition(targetPosition, liftPower);

            /* Claw Control */
            if(gamepad1.right_bumper || gamepad2.right_bumper) {
                elapsedTime.reset();
                drive.openClaw();
            } else if (gamepad1.left_bumper || gamepad2.left_bumper){
                drive.closeClaw();
            }

            if(gamepad1.dpad_up) {
                robot.lamp.setPower(1);
            } else if (gamepad1.dpad_down){
                robot.lamp.setPower(0);
            }

            if(robot.sensorCone.getDistance(DistanceUnit.CM) <= 5) {
                if(elapsedTime.time() >= robot.CONE_DISTANCE) {
                    drive.closeClaw();
                }
            }

            // 90 degree turn
            if(gamepad1.dpad_right) {
                TargetRotation = robot.TURN_ROTATION;
                OldRotation = robot.imu.getAbsoluteHeading();
                rotateEnabled = true;

                RFrotatePower = robot.TURN_SPEED;
                LFrotatePower = -robot.TURN_SPEED;
                LRrotatePower = -robot.TURN_SPEED;
                RRrotatePower = robot.TURN_SPEED;
            }
//            else if(gamepad1.dpad_left) {
//                TargetRotation = -robot.TURN_ROTATION;
//                OldRotation = -robot.imu.getAbsoluteHeading();
//                rotateEnabled = true;
//
//                RFrotatePower = -robot.TURN_SPEED;
//                LFrotatePower = robot.TURN_SPEED;
//                LRrotatePower = robot.TURN_SPEED;
//                RRrotatePower = -robot.TURN_SPEED;
//            }

            // Provide user feedback
            telemetry.addData("V1 = ", v1);
            telemetry.addData("elapsed time = ", elapsedTime.time());
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);
            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
            telemetry.addData("Theta = ", theta);
            telemetry.addData("Theta2 = ", theta);
            telemetry.addData("IMU Value: ", theta);
            telemetry.addData("robot rotation: ", OldRotation);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class