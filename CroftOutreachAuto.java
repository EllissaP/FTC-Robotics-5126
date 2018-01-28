package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//AHS Robotics
//Team 5126
//DICE
//Ellissa Peterson
//Croft Auto Outreach

@Autonomous(name="Auto Outreach", group="Concept")
@Disabled
public class CroftOutreachAuto extends OpMode
{
    private DcMotor LWheel;
    private DcMotor RWheel;

    public void init()
    {
        LWheel = hardwareMap.dcMotor.get("Left");
        RWheel = hardwareMap.dcMotor.get("Right");
    }

    public void start()
    {
        //first number tells Croft what way you move
        //second number tells Croft how long
        move(1, 4000);
















    }

    public void loop()
    {

    }
    public void move(int direction, long time)//-1 = left, 1 = right, 0 = forward, 2 = back
    {
        if(direction == -1)//left
        {
            RWheel.setPower(-.4);
            LWheel.setPower(-.4);
        }
        else if(direction == 0)//forward
        {
            RWheel.setPower(-.4);
            LWheel.setPower(.4);
        }
        else if(direction == 1)//right
        {
            RWheel.setPower(.4);
            LWheel.setPower(.4);
        }
        else if(direction == 2)//back
        {
            RWheel.setPower(.4);
            LWheel.setPower(-.4);
        }
        try
        {
            Thread.sleep(time);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        RWheel.setPower(0);
        LWheel.setPower(0);
        try//slight pause to make movements flow (less jerky)
        {
            Thread.sleep(150);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
}

