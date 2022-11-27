package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

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
        boolean TSEFlag = false;
        boolean fieldCentric = false;
        int targetPosition = 0;
        double cupPosition = 0;

        ElapsedTime currentTime = new ElapsedTime();
        double buttonPress = currentTime.time();

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ", "GOOD LUCK");
        telemetry.update();

        boolean shippingElement = false;
        boolean armDeployed = false;

        boolean clawOpen = true;

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().thirdAngle - 90;
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
            robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -power, power));
            robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -power, power));
            robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -power, power));
            robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -power, power));

            /*if (gamepad1.right_trigger > 0.1&&power < 1) {
                power +=.05;
            } else if (gamepad1.left_trigger > 0.1&&power > 0) {
                power -= 0.5;
            } */


            // Provide user feedback
            telemetry.addData("Left lift position:", robot.motorLeftLift.getCurrentPosition());
            telemetry.addData("Right lift position:", robot.motorRightLift.getCurrentPosition());
            telemetry.addData("Motor Left Rear:", robot.motorLR.getCurrentPosition());
            telemetry.addData("Motor Left Front:", robot.motorLF.getCurrentPosition());
            telemetry.addData("Motor Right Front:", robot.motorRF.getCurrentPosition());
            telemetry.addData("Motor Right Rear:", robot.motorRR.getCurrentPosition());
            telemetry.addData("V1 = ", v1);
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);
            telemetry.addData("IMU First Angle = ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("IMU Second Angle = ", robot.imu.getAngularOrientation().secondAngle);
            telemetry.addData("IMU Third Angle = ", robot.imu.getAngularOrientation().firstAngle);
            if( v1 != 0){
                telemetry.addData("Motor Left Front = ", v1);
            }
            if( v2 != 0) {
                telemetry.addData("Motor Right Front = ", v2);
            }
            if(v3 != 0){
                telemetry.addData("Motor Left Rear = ", v3);
            }
            if(v4 != 0) {
                telemetry.addData("Motor Right Rear = ", v4);
            }
            telemetry.addData("Left Stick X = ", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y = ", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X = ", gamepad1.right_stick_x);
            telemetry.addData("Right Stick Y = ", gamepad1.right_stick_y);
            telemetry.addData("Theta = ", theta);
            telemetry.addData("Theta2 = ", theta);
            telemetry.addData("IMU Value: ", theta);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class