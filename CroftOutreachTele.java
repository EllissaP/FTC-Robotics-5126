package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//AHS Robotics
//Team 5126
//DICE
//Ellissa Peterson
//Croft Teleop Outreach Code

@TeleOp(name="Croft Outreach TeleOp", group="Linear Opmode")
@Disabled
public class CroftOutreachTele extends OpMode
{
    private DcMotor LWheel;
    private DcMotor RWheel;
    private DcMotor ArmHeight;
    private DcMotor ArmRotation;
    private Servo Claw;

    private double leftP;
    private double rightP;
    private double armHeightP;
    private double drivePower;
    private double armPower;

    public void init()
    {
        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
        Claw = hardwareMap.servo.get("Claw");
        ArmRotation = hardwareMap.dcMotor.get("ArmRotation");
    }
    public void loop() {

        leftP = Range.clip(gamepad1.left_stick_y, -1, 1);
        rightP = Range.clip(gamepad1.right_stick_y, -1, 1);
        armHeightP = Range.clip(gamepad2.left_stick_y, -1, 1);

        if (gamepad2.a) {
            ArmRotation.setPower(.3);
        } else if (gamepad2.x) {
            ArmRotation.setPower(-.3);
        } else {
            ArmRotation.setPower(0);
        }

        if (gamepad2.b) {
            Claw.setPosition(1);
        } else {
            Claw.setPosition(0);
        }

        drivePower = Range.clip(gamepad1.right_trigger, 0, 1);
        armPower = Range.clip(gamepad2.left_trigger, 0, 1);

        //alter input data for motor powers
        LWheel.setPower(-leftP / ((3 * drivePower) + 2));
        RWheel.setPower(rightP / ((3 * drivePower) + 2));
        ArmHeight.setPower(-armHeightP / ((5 * armPower) + 2.5));

    }
}
