package org.firstinspires.ftc.teamcode.Libs;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

@Config
public class DriveClass {

    private HWProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;
    ElapsedTime runTime = new ElapsedTime();

    FtcDashboard dashboard;
    public static double kP = 0.6;
    public static double kI = 0.0003;
    public static double kD = 0.0001;
    public static double maxSpeed = 0.5;
    public static double minSpeed = 0.21;

    /*
     * Constructor method
     */
    public DriveClass(HWProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method

    /**
     * ###########################################################################################
     * ###########################################################################################
     * #######################      DRIVE CONTROLS      ##########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    /**
     * Method robotCorrect2
     *  -   This method is a revised version of robotCorrect. There were some behaviors that needed
     *      to be fixed, but didn't want to risk all of the programs. This one can be renamed
     *      robotCorrect and all instances of robotCorrect2 changed to robotCorrect.
     * @param power
     * @param heading
     * @param duration
     */
    public void driveByTime(double power, double heading, double duration) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runTime = new ElapsedTime();

        if(runTime.time() >= duration) active = false;

        while(opMode.opModeIsActive() && active) {
//            updateValues(action, initZ, theta, currentZ, zCorrection);

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runTime.time() >= duration) active = false;

            currentZ = getZAngle();
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
                    action = " initZ < currentZ";
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                    action = " initZ < currentZ";
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            if(RF > 1) RF = 1;
            else if (RF < -1) RF = -1;

            if(LF > 1) LF = 1;
            else if (LF < -1) LF = -1;

            if(LR > 1) LR = 1;
            else if (LR < -1) LR = -1;

            if(RR > 1) RR = 1;
            else if (RR < -1) RR = -1;

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

        }   // end of while loop

        motorsHalt();
    }   // close robotCorrect method

    /**
     *  Method: driveDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void driveDistance(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double strafeFactor = 1;

        double theta = Math.toRadians(90 + heading);
        double lfStart = 0;
        double lrStart = 0;
        double rfStart = 0;
        double rrStart = 0;

        lfStart = robot.motorLF.getCurrentPosition();
        lrStart = robot.motorLR.getCurrentPosition();
        rfStart = robot.motorRF.getCurrentPosition();
        rrStart = robot.motorRR.getCurrentPosition();

        if (heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if (initZ > 170 || initZ < -170){
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RF = Range.clip(RF, -1,1);
            LF = Range.clip(LF, -1,1);
            RR = Range.clip(RR, -1,1);
            LR = Range.clip(LR, -1,1);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);
            /*
            opMode.telemetry.addData("LF Start = ", lfStart);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Heading = ", heading);
            opMode.telemetry.addData("Calculated Distance = ", calcDistance(heading, rfStart, rrStart, lfStart, lrStart));
            opMode.telemetry.update();
             */

            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= (distance * strafeFactor)) active = false;
            opMode.idle();

        }   // end of while loop

        motorsHalt();

    }   // close driveDistance method

    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void PIDRotate(double targetAngle, double targetError){
        double integral = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = 0.006;
        double Ci = 0.003;
        double Cd = 0.00004;
        /* enable these for tuning
        double Cp = kP;
        double Ci = kI;
        double Cd = kD;
        double maxRotateSpeed = maxSpeed;
        double maxRotateSpeed = minSpeed;
         */
        double minRotateSpeed = 0.18;
        double maxRotateSpeed = 1;
        double rotationSpeed;
        double derivative = 0, lastError=0;
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        // check to see how far the robot is rotating to decide which gyro sensor value to use
        if(targetAngle > 90 || targetAngle < -90){
            error = gyro360(targetAngle) - targetAngle;
        } else {
            error = getZAngle() - targetAngle;
        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
                derivative = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative));
                lastError = error;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxRotateSpeed, maxRotateSpeed);

                if ((rotationSpeed > -0.25) && (rotationSpeed < 0)) {
                    rotationSpeed = -minRotateSpeed;
                } else if ((rotationSpeed < 0.25) && (rotationSpeed > 0)) {
                    rotationSpeed = minRotateSpeed;
                }

                RF = rotationSpeed;
                LF = -rotationSpeed;
                LR = -rotationSpeed;
                RR = rotationSpeed;

                setDrivePower(RF, LF, LR, RR);

                opMode.idle();

                // check to see how far the robot is rotating to decide which gyro sensor value to use
                if (targetAngle > 90 || targetAngle < -90) {
                    error = gyro360(targetAngle) - targetAngle;
                } else {
                    error = getZAngle() - targetAngle;
                }

            }   // end of while Math.abs(error)
            setDrivePower(0,0,0,0);
            maxRotateSpeed = maxRotateSpeed / 2;
            opMode.sleep(10);
//            opMode.idle();

            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
//            opMode.sleep(5);
            if (targetAngle > 90 || targetAngle < -90) {
                error = gyro360(targetAngle) - targetAngle;
            } else {
                error = -robot.imu.getAngles()[0] - targetAngle;
//                error = getZAngle() - targetAngle;
            }
        }

        // shut off the drive motors
        motorsHalt();

        totalTime = timeElapsed.time() - startTime;
        // post telemetry to FTC Dashboard
        dashTelemetry.put("p00 - PIDTurn Telemetry Data", "");
        dashTelemetry.put("p01 - PID IMU Angle X                  = ", robot.imu.getAngles()[0]);
        dashTelemetry.put("p02 - PID IMU Angle Y                  = ", robot.imu.getAngles()[1]);
        dashTelemetry.put("p03 - PID IMU Angle Z                  = ", robot.imu.getAngles()[2]);
        dashTelemetry.put("p04 - InitZ/targetAngle value      = ", targetAngle);
        dashTelemetry.put("p05 - Current Angle                = ", getZAngle());
        dashTelemetry.put("p06 - Angle Error                  = ", error);
        dashTelemetry.put("p07 - zCorrection/derivative Value = ", derivative);
        dashTelemetry.put("p08 - Turn Time                    = ", totalTime);
        dashTelemetry.put("p09 - Right Front                  = ", RF);
        dashTelemetry.put("p10 - Right Rear                   = ", RR);
        dashTelemetry.put("p11 - Left Front                   = ", LF);
        dashTelemetry.put("p12 - Right Rear                   = ", RR);
        dashboard.sendTelemetryPacket(dashTelemetry);
    }   //end of the PIDRotate Method

    /**
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     * @param targetAngle - Reference angle for the gyro sensor
     */
    public double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360


    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_CLOSE);
    }

    public void openClaw(){
        robot.servoGrabber.setPosition(robot.CLAW_OPEN);
    }

    public void fingerExtend() { robot.servoFinger.setPosition(robot.FINGER_OUT);}

    public void fingerRetract() { robot.servoFinger.setPosition(robot.FINGER_IN);}

    public void liftPosition(int liftPosition, double power) {
        robot.motorRightLift.setTargetPosition(liftPosition);
        robot.motorLeftLift.setTargetPosition(liftPosition);

        robot.motorRightLift.setPower(power);
        robot.motorLeftLift.setPower(power);
    }

    public void resetLift(double power){
        robot.motorRightLift.setTargetPosition(robot.LIFT_RESET);
        robot.motorLeftLift.setTargetPosition(robot.LIFT_RESET);

        robot.motorRightLift.setPower(power);
        robot.motorLeftLift.setPower(power);
    }

    /**
     *  Method: driveSimpleDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void driveSimpleDistance(double power, double heading, double distance) {
        boolean active = true;

        double theta = Math.toRadians(90 + heading);
        double lfStart = robot.motorLF.getCurrentPosition();
        double lrStart = robot.motorLR.getCurrentPosition();
        double rfStart = robot.motorRF.getCurrentPosition();
        double rrStart = robot.motorRR.getCurrentPosition();

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= distance) active = false;

        }   // end of while loop

        motorsHalt();
    }   // close simpleDriveDistance method

    /**
     * Sets power to all four drive motors
     * @param RF power for right front motor
     * @param LF power for left front motor
     * @param LR power for left rear motor
     * @param RR power for right rear motor
     */
    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.setPower(RF);
        robot.motorLF.setPower(LF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }   // end of the setDrivePower method

    /*
     * Method motorsHalt
     *  -   stops all drive motors
     */
    public void motorsHalt(){
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }   // end of motorsHalt method


    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    /**
     * Method getRunTime
     * @return runtime
     */
    public double getRunTime(){
        return runTime.time();
    }

    /**
     * Method getZAngle()
     *  -   This method returns the gyro position of the robot.
     * @return zAngle
     */
    public double getZAngle(){
        return (-robot.imu.getAbsoluteHeading());
        //return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method

    /**
     * Method getRPM()
     *  -   This method returns the ticks per second required for a specified RPM value
     * @param tickValue
     */
    public double getRPM(double tickValue){
        return (tickValue/28*60);
    }

    /**
     * Method rpmValue()
     *  -   This method returns the ticks per second required for a specified RPM value
     * @param targetRPM
     */
    public double rpmValue(double targetRPM){
        return (targetRPM*28/60);
    }


    /**
     * Method updateValues
     *  -   Prints the values of a number of parameters to the phone
     * @param action    -   Tells the user what method / action that the program is providing data for
     *
     */
    public void updateValues(String action, double initZ, double theta, double currentZ, double zCorrection){
        opMode.telemetry.addData("Current Action = ", action);
        opMode.telemetry.addData("InitZ/targetAngle value  = ", initZ);
        opMode.telemetry.addData("Theta/lastError Value= ", theta);
        opMode.telemetry.addData("CurrentZ/Error Value = ", currentZ);
        opMode.telemetry.addData("zCorrection/derivative Value = ", zCorrection);

        opMode.telemetry.addData("Right Front = ", RF);
        opMode.telemetry.addData("Left Front = ", LF);
        opMode.telemetry.addData("Left Rear = ", LR);
        opMode.telemetry.addData("Right Rear = ", RR);
        opMode.telemetry.update();
    }   // close updateValues method

    /**
     * Method: calcDistance
     * @param heading   - indicates the direction the robot is angled/heading
     * @param rfStart   - Right Front starting encoder value
     * @param rrStart   - Right Rear starting encoder value
     * @param lfStart   - Left Front starting encoder value
     * @param lrStart   - Left Rear starting encoder value
     */
    public double calcDistance(double heading, double rfStart, double rrStart, double lfStart, double lrStart){

        double distanceTraveled = 0;
        double rfEncoder = 0;
        double lfEncoder = 0;
        double rrEncoder = 0;
        double lrEncoder = 0;

        rfEncoder = robot.motorRF.getCurrentPosition();
        lfEncoder = robot.motorLF.getCurrentPosition();
        rrEncoder = robot.motorRR.getCurrentPosition();
        lrEncoder = robot.motorLR.getCurrentPosition();

        if((heading == 0) || (heading == 180) || (heading == 90) || (heading == -90)) {
            distanceTraveled = ((Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                    + Math.abs(rrStart-rrEncoder) + Math.abs(lrStart - lrEncoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);
        }

        if ((heading == 90) || (heading == -90)){
            distanceTraveled = ((Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                    + Math.abs(rrStart-rrEncoder) + Math.abs(lrStart - lrEncoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);
        }

        return Math.abs(distanceTraveled);
    }


    /*******************************************************************************************
     *******************************************************************************************
     ***************************** FTCLIB Classes - For Testing ********************************
     *******************************************************************************************
     *******************************************************************************************/


    /**
     *  Method: driveDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void newDriveDistance(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double strafeFactor = 1;

        double theta = Math.toRadians(90 + heading);

        /* reset the motor encoders */
        robot.motorLeftFront.resetEncoder();
        robot.motorLeftRear.resetEncoder();
        robot.motorRightFront.resetEncoder();
        robot.motorRightRear.resetEncoder();

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if (initZ > 170 || initZ < -170){
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                }
            }   // end of if currentZ != initZ

            /* Limit that value of the drive motors so that the power does not exceed 100% */
            RF = Range.clip(RF, -power,power);
            LF = Range.clip(LF, -power,power);
            RR = Range.clip(RR, -power,power);
            LR = Range.clip(LR, -power,power);

            /* Apply power to the drive wheels */
            newSetDrivePower(RF, LF, LR, RR);

            /* Print data to user screen
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Heading = ", heading);
            opMode.telemetry.addData("Calculated Distance = ", newCalcDistance(heading));
            opMode.telemetry.update();
             */

            if(newCalcDistance(heading) >= (distance * strafeFactor)) active = false;
            opMode.idle();

        }   // end of while loop

        newMotorsHalt();

    }   // close newDriveDistance method


    /**
     * Method: calcDistance
     * @param heading   - indicates the direction the robot is angled/heading
     */
    public double newCalcDistance(double heading){

        double distanceTraveled = 0;
        double strafeFactor = 1;

        double rfEncoder = robot.motorRightFront.getCurrentPosition();
        double lfEncoder = robot.motorLeftFront.getCurrentPosition();
        double rrEncoder = robot.motorRightRear.getCurrentPosition();
        double lrEncoder = robot.motorLeftRear.getCurrentPosition();

        if (heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        distanceTraveled = ((Math.abs(rfEncoder) + Math.abs(lfEncoder)
                + Math.abs(rrEncoder) + Math.abs(lrEncoder)) / 4) / (robot.DRIVE_TICKS_PER_INCH);

        return Math.abs(distanceTraveled * strafeFactor);
    }   // close newCalcDistance

    /**
     * Sets power to all four drive motors
     * @param rfPower power for right front motor
     * @param lfPower power for left front motor
     * @param lrPower power for left rear motor
     * @param rrPower power for right rear motor
     */
    public void newSetDrivePower(double rfPower, double lfPower, double lrPower, double rrPower){
        robot.motorRightFront.set(rfPower);
        robot.motorLeftFront.set(lfPower);
        robot.motorLeftRear.set(lrPower);
        robot.motorRightRear.set(rrPower);
    }   // end of the newSetDrivePower method

    /*
     * Method motorsHalt
     *  -   stops all drive motors
     */
    public void newMotorsHalt(){
        robot.motorRightFront.set(0);
        robot.motorLeftFront.set(0);
        robot.motorLeftRear.set(0);
        robot.motorRightRear.set(0);
    }   // end of newMotorsHalt method


    /**
     * Method: newPIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     */
    public void newPIDRotate(double targetAngle){
        PIDFController pidf = new PIDFController(robot.LIFT_kP, robot.LIFT_kI, robot.LIFT_kD, robot.LIFT_kF);

        double error = pidf.calculate(robot.imu.getAbsoluteHeading(), targetAngle);

        pidf.setSetPoint(targetAngle);

        // nested while loops are used to allow for a final check of an overshoot situation
        while (!pidf.atSetPoint() && opMode.opModeIsActive()) {
            error = pidf.calculate(robot.imu.getAbsoluteHeading(), targetAngle)/ 10;

            RF = Range.clip(error, -1, 1);
            LF = Range.clip(-error, -1, 1);
            LR = Range.clip(-error, -1, 1);
            RR = Range.clip(error, -1, 1);

            newSetDrivePower(RF, LF, LR, RR);

            /*
            opMode.telemetry.addData("IMU value: ", robot.imu.getAbsoluteHeading());
            opMode.telemetry.addData("Error: ", error);
            opMode.telemetry.addData("Target Angle: ", targetAngle);
            opMode.telemetry.update();
             */


        }   // end of while Math.abs(error)

        // shut off the drive motors
        newMotorsHalt();

    }   //end of the newPIDRotate Method

}   // close the driveMecanum class