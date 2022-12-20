package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@Autonomous(name = "Run Forward", group = "Competition")

public class RunForwardAuto extends LinearOpMode {
    private final static HWProfile robot = new HWProfile();

    @Override
    public void runOpMode(){

        robot.init(hardwareMap);

        telemetry.addData("Ready to Run: ","GOOD LUCK");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            robot.motorLeftFront.set(-.2);
            robot.motorLeftRear.set(-.2);
            robot.motorLeftRear.set(-.2);
            robot.motorRightRear.set(-.2);
            sleep(750);
            robot.motorLeftFront.set(0);
            robot.motorLeftRear.set(0);
            robot.motorRightFront.set(0);
            robot.motorRightRear.set(0);
            sleep(1000);
            }
            // Provide user feedback

            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
       // end of MSTeleop class