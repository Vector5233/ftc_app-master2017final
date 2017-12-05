/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name="RedLeft", group ="Concept")

public class RedLeft extends LinearOpMode {
    public static final String TAG = "Vuforia VuMark Sample";

    DcMotor frontLeft, frontRight, backLeft, backRight, liftMotor;
    Servo jewelKnocker, leftGrab, rightGrab;
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensor;
    float red, green, blue;
    Drive drive;
    RaymondAutonomousOpMode ray;

    float Lt, Rt;

    final double RIGHTGrab_OPEN = 0.8;
    final double RIGHTGrab_CLOSE = 0.4; //used to be 0.46
    final double LEFTGrab_OPEN = 0.2;
    final double LEFTGrab_CLOSE = 0.6; //used to be 0.54

    final double SPROCKET_RATIO = 2.0/3.0;
    final double TICKS_PER_INCH = SPROCKET_RATIO*(1120.0/(2*2*3.14159));

    final double RaiseArm = 1.0;
    final double LowerArm = 0.0;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ASC4bMD/////AAAAGQo793RFLUVDpV1hb5ufNBh/AXAtpjorAyvu24vcZ2AdlmilEYdD61K3xjN4XxdZmMc6NVCEdYQsF1bQxSyFUeUQ/ZzBvYYZnq4JTuLnGXGm1zhjNgRwNE0hWWY0IhipNoz+2ZUjGzWOxGq4hBB8LsVvQnaQR0Z/09iQ9p9zQ9eOD85Com5dXlxef6whuD/BRXyZSBeibi/zel9RKT9VCcCIsn7i0h62cApztPMq6NzBDFibiNsWDVoE83nw5utIPOGY4MsAyPHh27AhThKp83FAvlBE/RCDSrgUYRg2TOOFEu3uG7DVKjHrngLSRccN5eorXXVG7PdPoiHWTpSyVMaQSu/boDk6XgjgxwqGU/tB";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        initialization();
        waitForStart();
        relicTrackables.activate();
        sleep(1000);
        rightGrab.setPosition(RIGHTGrab_CLOSE);
        leftGrab.setPosition(LEFTGrab_CLOSE);
        sleep(500);
        liftMotor.setPower(1.0);
        sleep(500);
        liftMotor.setPower(0.0);
        ray. JewelKnocker();

        while (opModeIsActive()) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark){
                case LEFT: {
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,19);
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,12);
                    liftMotor.setPower(-1.0);
                    drive.DeliverGlyph();
                    break;
                }
                case RIGHT: {
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,34);
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,12);
                    liftMotor.setPower(-1.0);
                    drive.DeliverGlyph();
                    break;
                }
                case CENTER: {
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,26);
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,12);
                    liftMotor.setPower(-1.0);
                    drive.DeliverGlyph();
                    break;
                }
                default:{
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,26);
                    drive.TurnRightDegree(0.5,90);
                    drive.DriveForwardDistance(0.5,12);
                    liftMotor.setPower(-1.0);
                    drive.DeliverGlyph();
                    break;
                }

            }
        }
    }

    public void initialization () {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        liftMotor = hardwareMap.dcMotor.get ("liftMotor");


        colorSensor = hardwareMap.colorSensor.get("color");
        jewelKnocker = hardwareMap.servo.get("jewel");
        //jewelKnocker.setPosition(jewelKnocker_Raised);
        //RaiseJewelKnocker();

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

        drive = new Drive(frontLeft,frontRight,backLeft,backRight, liftMotor, gyro, leftGrab, rightGrab, this);
        ray = new RaymondAutonomousOpMode (drive, jewelKnocker, colorSensor, this);
    }
}