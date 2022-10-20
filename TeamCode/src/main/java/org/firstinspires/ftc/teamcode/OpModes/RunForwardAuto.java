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
        double v1, v2, v3, v4, robotAngle;
        double theta;
        double theta2 = 180;
        double r;
        double power=0.5;
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
            robot.motorLF.setPower(-.2);
            robot.motorLR.setPower(-.2);
            robot.motorRF.setPower(-.2);
            robot.motorRR.setPower(-.2);
            sleep(750);
            robot.motorLF.setPower(0);
            robot.motorLR.setPower(0);
            robot.motorRF.setPower(0);
            robot.motorRR.setPower(0);
            sleep(100000);
            }
            // Provide user feedback

            telemetry.update();

        }   // end of while(opModeIsActive)
    }   // end of runOpMode()
       // end of MSTeleop class