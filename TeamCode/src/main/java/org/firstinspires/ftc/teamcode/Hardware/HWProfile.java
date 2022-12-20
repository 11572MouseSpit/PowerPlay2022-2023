package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// test comment from Christopher

public class HWProfile {
    /* Public OpMode members. */
    public MotorEx motorLeftFront   =   null;
    public MotorEx motorRightFront  =   null;
    public MotorEx motorLeftRear    =   null;
    public MotorEx motorRightRear   =   null;
    public MotorEx motorRightLift   =   null;
    public MotorEx motorLeftLift    =   null;
    public MotorGroup motorsLift    =   null;

    public RevIMU imu =                 null;


    public DcMotor motorLF   = null;
    public DcMotor  motorLR  = null;
    public DcMotor  motorRF     = null;
    public DcMotor  motorRR    = null;
//    public DcMotor motorLeftLift = null;
//    public DcMotor motorRightLift = null;
//    public BNO055IMU imu = null;
    public Servo servoGrabber = null;
    public Servo servoFinger = null;

    public final double DRIVE_TICKS_PER_INCH = 40.6;      //temporary values => To be updated
    public final int LIFT_RESET = 0;
    public final int LIFT_LOW_JUNCTION = 380;
    public final int LIFT_MID_JUNCTION = 600;
    public final int LIFT_MAX_HEIGHT = 900;
    public final double LIFT_POSITION_TOLERANCE = 10;
    public final double LIFT_kP = 0.5;
    public final double LIFT_kI = 0.5;
    public final double LIFT_kD = 0.5;
    public final double LIFT_kF = 0.5;

    public final double STRAFE_FACTOR = 1.1;
    public final double FINGER_OUT = 0.4;
    public final double FINGER_IN = 0.6;



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

        // Define Motors utilizing FTCLib class
        motorLeftFront = new MotorEx(hwMap, "motorLF", Motor.GoBILDA.RPM_312);
        motorLeftFront.setRunMode(Motor.RunMode.RawPower);
        motorLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLeftFront.setInverted(false);
        motorLeftFront.resetEncoder();

        motorLeftRear = new MotorEx(hwMap, "motorLR", Motor.GoBILDA.RPM_312);
        motorLeftRear.setRunMode(Motor.RunMode.RawPower);
        motorLeftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setInverted(false);
        motorLeftRear.resetEncoder();

        motorRightFront = new MotorEx(hwMap, "motorRF", Motor.GoBILDA.RPM_312);
        motorRightFront.setRunMode(Motor.RunMode.RawPower);
        motorRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setInverted(true);
        motorRightFront.resetEncoder();

        motorRightRear = new MotorEx(hwMap, "motorRR", Motor.GoBILDA.RPM_312);
        motorRightRear.setRunMode(Motor.RunMode.RawPower);
        motorRightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setInverted(false);
        motorRightRear.resetEncoder();




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


        motorLeftLift = new MotorEx(hwMap, "motorLeftLift", Motor.GoBILDA.RPM_117);
        motorLeftLift.setRunMode(Motor.RunMode.PositionControl);
        motorLeftLift.setInverted(true);
        motorLeftLift.setPositionCoefficient(LIFT_kP);
        motorLeftLift.setTargetPosition(0);
        motorLeftLift.set(0);               // set motor power
        motorLeftLift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        motorRightLift = new MotorEx(hwMap, "motorLeftLift", Motor.GoBILDA.RPM_117);
        motorRightLift.setRunMode(Motor.RunMode.PositionControl);
        motorRightLift.setInverted(false);
        motorRightLift.setPositionCoefficient(LIFT_kP);
        motorRightLift.setTargetPosition(0);
        motorRightLift.set(0);               // set motor power
        motorRightLift.setPositionTolerance(LIFT_POSITION_TOLERANCE);

        MotorGroup liftMotors = new MotorGroup(motorLeftLift, motorRightLift);
        liftMotors.setRunMode(Motor.RunMode.PositionControl);
        liftMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

        servoGrabber = hwMap.get(Servo.class, "servoGrabber");
        servoFinger = hwMap.get(Servo.class, "servoFinger");


        // imu init
        imu = new RevIMU(hwMap);
        imu.init();

        /*
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

         */

    }
}  // end of HWProfile Class