package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// test comment from Christopher

public class HWProfile {
    /* Public OpMode members. */
    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;
    public DcMotor motorLeftLift = null;
    public DcMotor motorRightLift = null;
    public BNO055IMU imu = null;
    public Servo servoGrabber = null;

    public final double DRIVE_TICKS_PER_INCH = 40.6;      //temporary values => To be updated
    public final int LOW_JUNCTION_POSITION = 380;
    public final int MID_JUNCTION_POSITION = 600;
    public final int MAX_LIFT_POSITION = 1000;

    public final double STRAFE_FACTOR = 1.1;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HWProfile(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorDistance;

        // Define and Initialize Motors
        motorLF = hwMap.get(DcMotor.class, "motorLF");
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setPower(0);


        motorLR = hwMap.get(DcMotor.class, "motorLR");
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setPower(0);


        motorRF = hwMap.get(DcMotor.class, "motorRF");
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setPower(0);


        motorRR = hwMap.get(DcMotor.class, "motorRR");
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setDirection(DcMotor.Direction.REVERSE);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setPower(0);


        motorLeftLift = hwMap.get(DcMotor.class, "motorLeftLift");
        motorLeftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftLift.setTargetPosition(0);
        motorLeftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftLift.setPower(0);


        motorRightLift = hwMap.get(DcMotor.class, "motorRightLift");
        motorRightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRightLift.setTargetPosition(0);
        motorRightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightLift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoGrabber = hwMap.get(Servo.class, "servoGrabber");

        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

    }
}  // end of HWProfile Class