/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;


/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//AHS Robotics
//Team 5126
//DICE
//Ellissa Peterson
//Croft Teleop Test Code

@TeleOp(name="Tele test code", group="Linear Opmode")
@Disabled
public class CroftTeleTestCode extends LinearOpMode {

    private DcMotor LWheel;
    private DcMotor RWheel;
    private DcMotor ArmHeight;
    private DcMotor ArmRotation;
    private Servo Claw;
    private VoltageSensor volts;


    private long turnTime;//time to make a 90 degree turn
    private long parkTime;//time to park in box
    private int i;//vuforia counter

    int cameraMonitorViewId;

    private double leftP;
    private double rightP;
    private double armHeightP;
    private double drivePower;
    private double armPower;



    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;


    @Override public void runOpMode() {

        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
        Claw = hardwareMap.servo.get("Claw");
        ArmRotation = hardwareMap.dcMotor.get("ArmRotation");
        volts = hardwareMap.voltageSensor.get("Motor Controller 1");

        turnTime = 625;
        parkTime = 800;
        i = 0;

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AUab8lD/////AAAAGfdEGvojlULkpQTDLe8zdwREFmZqTFDUvuLi1b6FRcufb+FreEylImIJaeb5PJicszzZ7xRUFfpxp+uo1Elpr0fZPguNlkasbrN3ptWrr1wb7GqCfblzW1U/yYge9rOP5/Xc5UsjwpHI1xx42x78GS8ARqVCuHrsofoIx7y2sUaWfwZ5iK6wGJNl/fhArJ4PhG17kM2D3XFzKlcW+htqK4BWdIIeYqjbe76vvGGq6OJjqFMGM4ny8jOGr56GjtKNxHewsDkbm7sb8J9QVqZNUG0kHa5DBL9yzpaB7xLYZJNbN7gwDiLhWjGM58y61VP2pw06Zd2ZMCEETKhWHIAjZ3OWaS/CQSH6K46jXnzYOH/G";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;//can change to front if  needed
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); //can help in debugging, otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive())
        {
            leftP = Range.clip(gamepad1.left_stick_y, -1, 1);
            rightP = Range.clip(gamepad1.right_stick_y, -1, 1);
            armHeightP = Range.clip(gamepad2.left_stick_y, -1, 1);


            if (gamepad2.a)
            {
                ArmRotation.setPower(.5);
            }
            else if (gamepad2.x)
            {
                ArmRotation.setPower(-.5);
            }
            else
            {
                ArmRotation.setPower(0);
            }

            if (gamepad2.b)
            {
                Claw.setPosition(1);
            }
            else
            {
                Claw.setPosition(0);
            }

            drivePower = Range.clip(gamepad1.right_trigger, 0, 1);
            armPower = Range.clip(gamepad2.left_trigger, 0, 1);

            //alter input data for motor powers
            LWheel.setPower(-leftP / ((3 * drivePower) + 1));
            RWheel.setPower(rightP / ((3 * drivePower) + 1));
            ArmHeight.setPower(-armHeightP / ((3 * armPower) + 2.5));

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

                telemetry.addData("VuMark", vuMark);

                //extra code for position if needed later for precision
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }

            }
            telemetry.update();
        }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
