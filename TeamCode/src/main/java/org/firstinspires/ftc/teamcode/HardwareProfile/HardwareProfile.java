// change comments

/*
12/7/21 added turret continuous servos: turret1 & turret2
        add turretEncoder - encoder added to inTakeMotor referneced via
        turretEncoder object
12/10/21 renamed turret servos
*/
package org.firstinspires.ftc.teamcode.HardwareProfile;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareProfile {

    /*
     * Constants
     */

    public final double DRIVE_TICKS_PER_INCH = 29.0;

    //drive control constants
    public final double DRIVE_MULTIPLIER=0.5;
    public final double REVERSE_MULTIPLIER=0.75;
    public final double TURN_MULTIPLIER=0.75;

    /*
     * Hardware devices
     */

    //Motors
    public DcMotor motorRF = null;  // Right Front Drive Motor
    public DcMotor motorLF = null;  // Left Front Drive Motor
    public DcMotor motorLR = null;  // Left Rear  Drive Motor
    public DcMotor motorRR = null;  // Right Rear Drive Motor

    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor

    //Servos
//    public Servo intakeDeployBlue = null; //Right intake deploy servo

    //public distance sensors;
//    public DistanceSensor sensorDistPink=null;

    // public WebcamName webcam = null;

    /* Constructor */
    public HardwareProfile() {

    }   // end of HardwareProfile method

    public void init(HardwareMap hwMap) {

//initialize DcMotor motors



//initialize servos
//        intakeDeployPink = hwMap.servo.get("intakeDeployPink");

        motorLF = hwMap.dcMotor.get("motorLF");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setPower(0);

        motorLR = hwMap.dcMotor.get("motorLR");
        motorLR.setDirection(DcMotor.Direction.REVERSE);
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setPower(0);

        motorRF = hwMap.dcMotor.get("motorRF");
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setPower(0);

        motorRR = hwMap.dcMotor.get("motorRR");
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setPower(0);

//initialize sensors
//        sensorDistBlue=hwMap.get(DistanceSensor.class, "sensorDistBlue");
//        sensorDistPink=hwMap.get(DistanceSensor.class, "sensorDistPink");

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        /* Webcam device will go here */
        // webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }   // end of init() method

}