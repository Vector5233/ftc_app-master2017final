package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by CCA on 8/16/2017.
 */

@TeleOp(name="RevRoboticsOp", group = "myGroup")
public class RevRoboticsOp extends OpMode {


    DcMotor frontLeft, frontRight, backLeft, backRight, liftMotor;
    ColorSensor colorSensor;
    float red, green, blue;
    Servo leftGrab,rightGrab;
    Servo jewelKnocker;
    ModernRoboticsI2cGyro gyro;
    float Lt,Rt;


    final double RIGHTGrab_OPEN = 0.8;
    final double RIGHTGrab_CLOSE = 0.35; //used to be 0.4
    final double LEFTGrab_OPEN = 0.2;
    final double LEFTGrab_CLOSE = 0.65; //used to be 0.6

    final double JEWEL_UP = 0;
    final double JEWEL_DOWN = 0+0.091; //new number: 0.222

    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        //gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        //colorSensor = hardwareMap.colorSensor.get("color");
        jewelKnocker = hardwareMap.servo.get("jewel");
        jewelKnocker.setPosition(JEWEL_UP);  //JEWEL_UP

        rightGrab = hardwareMap.servo.get("rightGrab");
        leftGrab = hardwareMap.servo.get("leftGrab");
        rightGrab.setPosition(RIGHTGrab_OPEN);
        leftGrab.setPosition(LEFTGrab_OPEN);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop () {
// 2 Drive: movement + tasks
        /*red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        */


        frontLeft.setPower(-gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x);
        frontRight.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x);
        backLeft.setPower(-gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x);
        backRight.setPower(-gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x);

        /*telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.addData("Blue: ", blue);*/
        //telemetry.addData("gyro: ",gyro.getIntegratedZValue());
        telemetry.addData("Front Left:", gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x);
        telemetry.addData("Back Right:", gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x);

        if (gamepad1.dpad_up){

            frontLeft.setPower(1.0);
            frontRight.setPower(1.0);
            backLeft.setPower(1.0);
            backRight.setPower(1.0);

        }

        if (gamepad1.dpad_down){

            frontLeft.setPower(-1.0);
            frontRight.setPower(-1.0);
            backLeft.setPower(-1.0);
            backRight.setPower(-1.0);
        }

        if (gamepad1.dpad_left){

            frontLeft.setPower(-1.0);
            frontRight.setPower(1.0);
            backLeft.setPower(1.0);
            backRight.setPower(-1.0);


        }

        if (gamepad1.dpad_right){

            frontLeft.setPower(1.0);
            frontRight.setPower(-1.0);
            backLeft.setPower(-1.0);
            backRight.setPower(1.0);

        }



        if (gamepad2.a) {
            jewelKnocker.setPosition(JEWEL_DOWN);
        }
        else if (gamepad2.b) {
            jewelKnocker.setPosition(JEWEL_UP);
        }


        // trivial change
        if (gamepad2.dpad_up) {
            liftMotor.setPower(1.0);
        }
        else if (gamepad2.dpad_down) {
            liftMotor.setPower(-1.0);
        }
        else {
            liftMotor.setPower(0);
        }


        if (gamepad2.left_bumper) {
            rightGrab.setPosition(RIGHTGrab_OPEN);
            leftGrab.setPosition(LEFTGrab_OPEN);
        }

        if (gamepad2.right_bumper) {
            rightGrab.setPosition(RIGHTGrab_CLOSE);
            leftGrab.setPosition(LEFTGrab_CLOSE);
        }

    }

}
