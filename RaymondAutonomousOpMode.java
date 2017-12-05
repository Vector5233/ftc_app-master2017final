package org.firstinspires.ftc.teamcode; /**
 * Created by CCA on 10/26/2017.
 */


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class RaymondAutonomousOpMode extends Object {
    Servo jewelKnocker, leftGrab, rightGrab;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    ColorSensor colorSensor;
    Drive drive;
    LinearOpMode opmode;
    float red, green, blue;

    final double JEWEL_UP = 0;
    final double JEWEL_DOWN = 0+0.091;

    public RaymondAutonomousOpMode (Drive D, Servo JK, ColorSensor CS, LinearOpMode L) {
        drive = D;
        jewelKnocker = JK;
        colorSensor = CS;
        opmode = L;
    }
    public void JewelKnocker(){
        LowerJewelKnocker();
        opmode.sleep(2000);
        opmode.telemetry.addData("Red: ", colorSensor.red());
        opmode.telemetry.addData("Blue: ", colorSensor.blue());
        opmode.telemetry.update();
        if (colorSensor.red() < colorSensor.blue()) {
            drive.TurnLeftDegree(0.3,21);
            drive.TurnRightDegree(0.3,21);
            RaiseJewelKnocker();
        } else {
            drive.TurnRightDegree(0.3,21);
            drive.TurnLeftDegree(0.3,21);
            RaiseJewelKnocker();
        }
    }

    public void LowerJewelKnocker(){
        jewelKnocker.setPosition(JEWEL_DOWN);
    }

    public void RaiseJewelKnocker(){
        jewelKnocker.setPosition(JEWEL_UP);
    }
}




