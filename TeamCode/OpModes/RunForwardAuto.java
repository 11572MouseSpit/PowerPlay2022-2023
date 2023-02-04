package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@Autonomous(name = "Run Forward", group = "Competition")

public class RunForwardAuto extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();
    StandardTrackingWheelLocalizer drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        drive.setPoseEstimate(new Pose2d(10, 10, Math.toRadians(90)));

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
//            robot.motorLeftFront.set(-.2);
//            robot.motorLeftRear.set(-.2);
//            robot.motorLeftRear.set(-.2);
//            robot.motorRightRear.set(-.2);
//            sleep(750);
//            robot.motorLeftFront.set(0);
//            robot.motorLeftRear.set(0);
//            robot.motorRightFront.set(0);
//            robot.motorRightRear.set(0);
//            sleep(1000);
            Vector2d myVector = new Vector2d(10, -5);

            Pose2d myPose = new Pose2d(10, -5, Math.toRadians(90));
        }
            // Provide user feedback

            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
       // end of MSTeleop class