package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "test2020")
public class MecanumTest extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode()
    {
       robot = new Robot(hardwareMap, telemetry);
       waitForStart();
       robot.moveForwardInches(5,.5);
       robot.turnToAng(90,1,20,10000);
    }

}
