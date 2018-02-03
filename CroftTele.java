package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//AHS Robotics
//Team 5126
//DICE
//Ellissa Peterson
//Preston Shives
//Croft Teleop

@TeleOp(name="Croft TeleOp", group="Linear Opmode")
@Disabled
public class CroftTele extends OpMode
{
    private DcMotor LWheel;
    private DcMotor RWheel;
    private DcMotor ArmHeight;
    private DcMotor ArmRotation;
    private Servo Claw;
    private Servo ColorArm;

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
        ColorArm = hardwareMap.servo.get("Color Arm");
    }
    public void loop() {

        leftP = Range.clip(gamepad1.left_stick_y, -1, 1);
        rightP = Range.clip(gamepad1.right_stick_y, -1, 1);
        armHeightP = Range.clip(gamepad2.left_stick_y, -1, 1);
        drivePower = Range.clip(gamepad1.right_trigger, 0, 1);
        armPower = Range.clip(gamepad2.right_stick_y, -1, 1);


        ArmRotation.setPower(.4 * armPower);

        if (gamepad2.b)
        {
            Claw.setPosition(1);
        }
        else
        {
            Claw.setPosition(0);
        }

        if (gamepad2.y)
        {
            ColorArm.setPosition(.1);
        }
        else
        {
            ColorArm.setPosition(.5);
        }

        //alter input data for motor powers
        LWheel.setPower((leftP / 2.1) * ((drivePower * .5) + 1));
        RWheel.setPower((-rightP / 2.1) * ((drivePower * .5) + 1));
        ArmHeight.setPower(-armHeightP / ((3 * armPower) + 2.5));
    }
}