package org.firstinspires.ftc.teamcode;

/**
 * Created by CCA on 10/27/2017.
 */

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by CCA on 10/26/2017.
 */

@Disabled
@Autonomous(name = "RevAuto")

public class RevAuto extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, liftMotor;
    ColorSensor colorSensor;
    float red, green, blue;
    Servo leftGrab, rightGrab;
    Servo jewelKnocker;
    ModernRoboticsI2cGyro gyro;
    Drive drive;
    RaymondAutonomousOpMode ray;

    final double SPROCKET_RATIO = 2.0/3.0;
    final double TICKS_PER_INCH = SPROCKET_RATIO*(1120.0/(2*2*3.14159));

    float Lt, Rt;
    final double RIGHTGrab_OPEN = 0.8;
    final double RIGHTGrab_CLOSE = 0.4; //used to be 0.46
    final double LEFTGrab_OPEN = 0.2;
    final double LEFTGrab_CLOSE = 0.6; //used to be 0.54

    final double JEWEL_UP = 0;
    final double JEWEL_DOWN = 0+0.091;


    @Override
    public void runOpMode() throws InterruptedException {

        initialization();
        waitForStart();
        /*drive.StrafeLeftDistance(0.5,12);
        sleep(5000);
        drive.StrafeRightDistance(0.5,12);
        sleep(5000);
        drive.StopDriving(); //redundant
        drive.DriveForwardDistance(0.5,12);
        sleep(5000);
        drive.DriveBackwardDistance(0.5,12);
        sleep(5000);
        drive.TurnLeftDegree(0.5,90);
        sleep(5000);
        drive.TurnRightDegree(0.5,90);*/
        drive.DeliverGlyph();
    }
/*Time magic number: 1000 = 1 second*/


    public void initialization () {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        liftMotor = hardwareMap.dcMotor.get ("liftMotor");


        colorSensor = hardwareMap.colorSensor.get("color");
        jewelKnocker = hardwareMap.servo.get("jewel");
        gyro=hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        //jewelKnocker.setPosition(jewelKnocker_Raised);
        RaiseJewelKnocker();

        rightGrab = hardwareMap.servo.get("rightGrab");
        leftGrab = hardwareMap.servo.get("leftGrab");
        rightGrab.setPosition(RIGHTGrab_OPEN);
        leftGrab.setPosition(LEFTGrab_OPEN);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gyro.calibrate();
        while (opModeIsActive() && gyro.isCalibrating()) {
            idle();
        }

        drive = new Drive(frontLeft,frontRight,backLeft,backRight, liftMotor, gyro, leftGrab, rightGrab, this);
        ray = new RaymondAutonomousOpMode (drive, jewelKnocker, colorSensor, this);


    }

    public void LowerJewelKnocker(){
        jewelKnocker.setPosition(JEWEL_DOWN);
    }

    public void RaiseJewelKnocker(){
        jewelKnocker.setPosition(JEWEL_UP);

    }
}


