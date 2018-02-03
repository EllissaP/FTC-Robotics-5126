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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

//AHS Robotics
//Team 5126
//DICE
//Ellissa Peterson
//Croft Autonomous for left blue alliance

@Autonomous(name="Concept: Auto Blue Left", group ="Concept")
@Disabled
public class CroftAutoBlueLeft extends LinearOpMode
{
    private DcMotor LWheel;
    private DcMotor RWheel;
    private DcMotor ArmHeight;
    private DcMotor ArmRotation;
    private Servo Claw;
    private Servo ColorArm;
    private com.qualcomm.robotcore.hardware.ColorSensor ColorSensor;

    private int i;//vuforia counter

    int cameraMonitorViewId;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.26;

    @Override public void runOpMode() {

        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
        Claw = hardwareMap.servo.get("Claw");
        ArmRotation = hardwareMap.dcMotor.get("ArmRotation");
        ColorArm = hardwareMap.servo.get("Color Arm");
        ColorSensor = hardwareMap.colorSensor.get("Color Sensor");

        i = 0;

        RWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        Claw.setPosition(.1);//set claw on block
        ColorArm.setPosition(.5);//starts color arm still
        ColorSensor.enableLed(true);

        while (opModeIsActive() && i < 1)
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark", "%s visible", vuMark);

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
            else
            {
                telemetry.addData("VuMark", "not visible");
                ColorArm.setPosition(.8);//0 or .8
                try//pause for flow
                {
                    Thread.sleep(1500);
                }
                catch (InterruptedException ex)
                {
                    Thread.currentThread().interrupt();
                }
                if(ColorSensor.red() > .1)
                {
                    encoderDrive(DRIVE_SPEED, -2, 2, 3.0);//move forward to knock off red ball
                    ColorArm.setPosition(.1);//0 or .8
                    try//pause for flow
                    {
                        Thread.sleep(400);
                    }
                    catch (InterruptedException ex)
                    {
                        Thread.currentThread().interrupt();
                    }
                }
                else
                {
                    encoderDrive(DRIVE_SPEED, 2.3, -2.3, 3.0);//moves back to knock off red ball
                    ColorArm.setPosition(.1);//0 or .8
                    try//pause for flow
                    {
                        Thread.sleep(400);
                    }
                    catch (InterruptedException ex)
                    {
                        Thread.currentThread().interrupt();
                    }
                    encoderDrive(DRIVE_SPEED, -3, 3, 3.0);//move forward to starting position
                }
                encoderDrive(DRIVE_SPEED, 26,  -26, 5.0);//drive back
                ColorArm.setPosition(.8);//puts color arm down
                try//pause for flow
                {
                    Thread.sleep(400);
                }
                catch (InterruptedException ex)
                {
                    Thread.currentThread().interrupt();
                }
                lift(1, 600); //set block arm up straight
                ColorArm.setPosition(.1);//tuck color arm away
                try//pause for flow
                {
                    Thread.sleep(400);
                }
                catch (InterruptedException ex)
                {
                    Thread.currentThread().interrupt();
                }
                i++;
            }
            telemetry.update();
        }

    }
    public void lift(int component, long time)//1 = rotate forward, -1 = rotate back, 2 = move up, 0 = move down
    {
        if(component == 1)//rotateF
        {
            ArmRotation.setPower(.5);
        }
        if(component == -1)//rotateB
        {
            ArmRotation.setPower(.5);
        }
        else if(component == 2)//up
        {
            ArmHeight.setPower(.4);
        }
        else if(component == 0)//down
        {
            ArmHeight.setPower(.4);
        }
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        ArmRotation.setPower(0);
        ArmHeight.setPower(0);
        try//pause for flow
        {
            Thread.sleep(300);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = RWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            LWheel.setTargetPosition(newLeftTarget);
            RWheel.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            RWheel.setPower(Math.abs(speed));
            LWheel.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (LWheel.isBusy() && RWheel.isBusy()))
            {
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", LWheel.getCurrentPosition(), RWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LWheel.setPower(0);
            RWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            LWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(300);//pause after each move to smooth motion
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
