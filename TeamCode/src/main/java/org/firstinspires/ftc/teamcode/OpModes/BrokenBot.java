package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@TeleOp(name = "Broken Bot", group = "Competition")

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode() {
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power = 1;
        double rightX, rightY;
        boolean fieldCentric = false;
        int targetPosition = 0;
        LinearOpMode opMode = this;

        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        robot.motorRightLift.setTargetPosition(0);
        robot.motorLeftLift.setTargetPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAbsoluteHeading() - 90;
                //robot.imu.getAngularOrientation().thirdAngle - 90;
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

            if (gamepad1.dpad_up){
                v1 = 1;
            } else if (gamepad1.dpad_down){
                v3 = 1;
            } else if (gamepad1.dpad_left) {
                v2 = 1;
            } else if (gamepad1.dpad_right) {
                v4 = 1;
            }
            robot.motorLeftFront.set(com.qualcomm.robotcore.util.Range.clip((v1), -power, power));
            robot.motorRightFront.set(com.qualcomm.robotcore.util.Range.clip((v2), -power, power));
            robot.motorLeftRear.set(com.qualcomm.robotcore.util.Range.clip((v3), -power, power));
            robot.motorRightRear.set(com.qualcomm.robotcore.util.Range.clip((v4), -power, power));


            if (gamepad1.right_trigger > 0.1 && power < 1) {
                power +=.05;
            } else if (gamepad1.left_trigger > 0.1 && power > 0) {
                power -= 0.5;
            }

            /*  LIFT CONTROL  */
            if(gamepad1.a){
                targetPosition = robot.LIFT_RESET;
                robot.servoFinger.setPosition(robot.FINGER_IN);
            } else if(gamepad1.b){
                targetPosition = robot.LIFT_LOW_JUNCTION;
                robot.servoFinger.setPosition(robot.FINGER_OUT);
            } else if(gamepad1.x) {
                targetPosition = robot.LIFT_MID_JUNCTION;
                robot.servoFinger.setPosition(robot.FINGER_OUT);
            } else if(gamepad1.y) {
                targetPosition = robot.LIFT_MAX_HEIGHT;
                robot.servoFinger.setPosition(robot.FINGER_OUT);
            }

            if (gamepad1.left_trigger > 0.1){
                targetPosition = targetPosition + 20;
            } else if (gamepad1.right_trigger > 0.1) {
                targetPosition = targetPosition - 20;
            } else if (gamepad1.dpad_up) {
                robot.servoFinger.setPosition(robot.FINGER_OUT);
            } else if (gamepad1.dpad_down) {
                robot.servoFinger.setPosition(robot.FINGER_IN);
            }

            /* Limit the range of the lift so as not to damage the robot */
            targetPosition = Range.clip(targetPosition, robot.LIFT_RESET, robot.LIFT_MAX_HEIGHT);

            drive.liftPosition(targetPosition, robot.LIFT_POWER);

            /* Claw Control */
            if(gamepad1.right_bumper) {
                drive.openClaw();
            } else if (gamepad1.left_bumper){
                drive.closeClaw();
            }

            if(gamepad2.dpad_right){
                drive.PIDRotate(-90, 2);
            }
            if (gamepad2.dpad_left){
                drive.PIDRotate(90, 2);
            }
            // Provide user feedback
            telemetry.addData("Target Lift Position: ", targetPosition);
//            telemetry.addData("Lift Motors positions: ", robot.motorsLift.getPositions());
            telemetry.addData("Finger Position: ", robot.servoFinger.getPosition());
            telemetry.addData("Left lift position: ", robot.motorLeftLift.getCurrentPosition());
            telemetry.addData("Right lift position: ", robot.motorRightLift.getCurrentPosition());
            telemetry.addData("Motor Left Rear: ", robot.motorLeftRear.getCurrentPosition());
            telemetry.addData("Motor Left Front: ", robot.motorLeftFront.getCurrentPosition());
            telemetry.addData("Motor Right Front: ", robot.motorRightFront.getCurrentPosition());
            telemetry.addData("Motor Right Rear: ", robot.motorRightRear.getCurrentPosition());
            telemetry.addData("IMU Absolute Heading: ", robot.imu.getAbsoluteHeading());
            telemetry.addData("IMU Heading: ", robot.imu.getHeading());
            telemetry.addData("IMU Angles?: ", robot.imu.getAngles());
            if( v1 != 0){
                telemetry.addData("Motor Left Front: ", v1);
            }
            if( v2 != 0) {
                telemetry.addData("Motor Right Front: ", v2);
            }
            if(v3 != 0){
                telemetry.addData("Motor Left Rear: ", v3);
            }
            if(v4 != 0) {
                telemetry.addData("Motor Right Rear: ", v4);
            }
            telemetry.addData("Left Stick X: ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y: ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X: ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y: ", gamepad1.right_stick_y);
            telemetry.addData("Theta: ", theta);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of BrokenBot class