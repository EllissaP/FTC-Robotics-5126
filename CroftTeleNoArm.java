package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

//AHS Robotics
//Team 5126
//DICE
//Ellissa Peterson
//Preston Shives
//Croft Teleop without arm

@TeleOp(name="Croft TeleOp", group="Linear Opmode")
@Disabled
public class CroftTeleNoArm extends OpMode
{
    private DcMotor LWheel;
    private DcMotor RWheel;

    private double leftP;
    private double rightP;
    private double drivePower;

    public void init()
    {
        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
    }
    public void loop() {

        leftP = Range.clip(gamepad1.left_stick_y, -1, 1);
        rightP = Range.clip(gamepad1.right_stick_y, -1, 1);

        drivePower = Range.clip(gamepad1.right_trigger, 0, 1);

        LWheel.setPower(-leftP / ((3 * drivePower) + 1));
        RWheel.setPower(rightP / ((3 * drivePower) + 1));
    }
}
