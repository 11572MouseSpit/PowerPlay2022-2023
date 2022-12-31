package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile;
import org.firstinspires.ftc.teamcode.Libs.DriveClass;

@Config
@TeleOp(name = "Broken Bot", group = "Competition")

public class BrokenBot extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    FtcDashboard dashboard;
    public static double l1_CLAW_OPEN = robot.CLAW_OPEN;
    public static double l2_CLAW_CLOSE = robot.CLAW_CLOSE;
    public static int l3_LIFT_JUNCTION_HIGH = robot.LIFT_HIGH_JUNCTION;
    public static int l4_LIFT_JUNCTION_MID = robot.LIFT_MID_JUNCTION;
    public static int l5_LIFT_JUNCTION_LOW = robot.LIFT_LOW_JUNCTION;
    public static int l6_LIFT_POSITION = 0;
    public static double l7_Lift_Up_Power = robot.LIFT_POWER_UP;
    public static double l8_Lift_Down_Power = robot.LIFT_POWER_DOWN;
    public static double l9_Finger_OUT = robot.FINGER_OUT;
    public static double l10_Finger_IN = robot.FINGER_IN;

    @Override
    public void runOpMode() {
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        boolean detected = false;
        double power = 1;
        double rightX, rightY;
        boolean fieldCentric = false;
        int targetPosition = 0;
        LinearOpMode opMode = this;
        ElapsedTime currentTime = new ElapsedTime();
        double timeStamp =0;
        double liftPower = robot.LIFT_POWER_DOWN;

        robot.init(hardwareMap);

        DriveClass drive = new DriveClass(robot, opMode);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

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
                robot.servoFinger.setPosition(l10_Finger_IN);
                liftPower = l8_Lift_Down_Power;
            } else if(gamepad1.b){
                targetPosition = l5_LIFT_JUNCTION_LOW;
                robot.servoFinger.setPosition(l9_Finger_OUT);
                liftPower = l7_Lift_Up_Power;
            } else if(gamepad1.x) {
                targetPosition = l4_LIFT_JUNCTION_MID;
                robot.servoFinger.setPosition(l9_Finger_OUT);
                liftPower = l7_Lift_Up_Power;
            } else if(gamepad1.y) {
                targetPosition = l3_LIFT_JUNCTION_HIGH;
                robot.servoFinger.setPosition(l9_Finger_OUT);
                liftPower = l7_Lift_Up_Power;
            }

            if (gamepad1.left_trigger > 0.1){
                targetPosition = targetPosition + 20;
                liftPower = l7_Lift_Up_Power;
            } else if (gamepad1.right_trigger > 0.1) {
                targetPosition = targetPosition - 20;
                liftPower = l8_Lift_Down_Power;
            } else if (gamepad1.dpad_up) {
                robot.servoFinger.setPosition(l9_Finger_OUT);
            } else if (gamepad1.dpad_down) {
                robot.servoFinger.setPosition(l10_Finger_IN);
            }

            if(gamepad2.a){
                targetPosition = l6_LIFT_POSITION;
                liftPower = l8_Lift_Down_Power;
            }

            /* Limit the range of the lift so as not to damage the robot */
            targetPosition = Range.clip(targetPosition, robot.LIFT_RESET, robot.LIFT_MAX_HEIGHT);

            drive.liftPosition(targetPosition, liftPower);

            /* Claw Control */
            if(robot.sensorCone.getDistance(DistanceUnit.INCH) < 2){
                detected = true;
            } else detected = false;
            if(gamepad1.right_bumper) {
                robot.servoGrabber.setPosition(l1_CLAW_OPEN);
                timeStamp = currentTime.time();
            } else if ((gamepad1.left_bumper || detected) && (currentTime.time()-timeStamp) > 1){
                robot.servoGrabber.setPosition(l2_CLAW_CLOSE);
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


            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngles()[0]);
            dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngles()[2]);
            dashTelemetry.put("04 - Lift Right Encoder Value = ", robot.motorRightLift.getCurrentPosition());
            dashTelemetry.put("05 - Lift Left Encoder Value = ", robot.motorLeftLift.getCurrentPosition());
            dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
            dashTelemetry.put("07 - GP1.Button.A = ", "RESET LIFT");
            dashTelemetry.put("08 - GP1.Button.B = ", "LIFT LOW JUNCTION");
            dashTelemetry.put("09 - GP1.Button.X = ", "LIFT MID JUNCTION");
            dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
            dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
            dashTelemetry.put("12 - Lift Power = ", liftPower);
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
}       // end of BrokenBot class