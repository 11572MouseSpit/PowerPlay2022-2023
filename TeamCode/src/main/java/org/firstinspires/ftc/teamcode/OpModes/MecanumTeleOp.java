package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

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
        boolean TSEFlag = false;
        boolean fieldCentric = false;
        int targetPosition = 0;
        double cupPosition = 0;

        ElapsedTime currentTime= new ElapsedTime();
        double buttonPress = currentTime.time();

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        boolean shippingElement=false;
        boolean armDeployed=false;

        waitForStart();

        while (opModeIsActive()) {

            /*******************************************
             ****** Mecanum Drive Control section ******
             *******************************************/
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
                theta = robot.imu.getAngularOrientation().firstAngle + 90;
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

            robot.motorLF.setPower(com.qualcomm.robotcore.util.Range.clip((v1), -power, power));
            robot.motorRF.setPower(com.qualcomm.robotcore.util.Range.clip((v2), -power, power));
            robot.motorLR.setPower(com.qualcomm.robotcore.util.Range.clip((v3), -power, power));
            robot.motorRR.setPower(com.qualcomm.robotcore.util.Range.clip((v4), -power, power));

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
            if(gamepad1.y){
                robot.motorRightLift.setTargetPosition(robot.MAX_LIFT_POSITION);
                robot.motorLeftLift.setTargetPosition(robot.MAX_LIFT_POSITION);
                robot.motorLeftLift.setPower(0.9);
                robot.motorRightLift.setPower(0.9);
            } else if (gamepad1.b) {
                robot.motorRightLift.setTargetPosition(robot.MID_JUNCTION_POSITION);
                robot.motorLeftLift.setTargetPosition(robot.MID_JUNCTION_POSITION);
                robot.motorLeftLift.setPower(.9);
                robot.motorRightLift.setPower(0.9);
            }  else if (gamepad1.a) {
                robot.motorRightLift.setTargetPosition(robot.LOW_JUNCTION_POSITION);
                robot.motorLeftLift.setTargetPosition(robot.LOW_JUNCTION_POSITION);
                robot.motorLeftLift.setPower(0.9);
                robot.motorRightLift.setPower(0.9);
            } else if (gamepad1.right_trigger>0.1) {
                robot.motorRightLift.setTargetPosition(0);
                robot.motorLeftLift.setTargetPosition(0);
                robot.motorLeftLift.setPower(0.3);
                robot.motorRightLift.setPower(0.3);
            } else if (gamepad1.left_trigger>0.1) {
                robot.motorLeftLift.setPower(0.9);
                robot.motorRightLift.setPower(0.9);
            } else {
                robot.motorLeftLift.setPower(0);
                robot.motorRightLift.setPower(0);
            }
            if(gamepad1.right_bumper){
                robot.servoGrabber.setPosition(0.6);
            }
            if(gamepad1.left_bumper) {
                robot.servoGrabber.setPosition(0.3);
            }


            // Provide user feedback
            telemetry.addData("V1 = ", v1);
            telemetry.addData("V2 = ", v2);
            telemetry.addData("V3 = ", v3);
            telemetry.addData("V4 = ", v4);
            telemetry.addData("Motor Left Lift = ", robot.motorLeftLift.getCurrentPosition());
            telemetry.addData("Motor Right Lift = ", robot.motorRightLift.getCurrentPosition());
            telemetry.addData("Theta = ", theta);
            telemetry.addData("Theta2 = ", theta);
            telemetry.addData("IMU Value: ", theta);
            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of MSTeleop class