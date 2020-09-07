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
       robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
       waitForStart();
       robot.moveForwardInches(2,.5);
       robot.turnToAng(45,.5,50,10000);
       robot.moveAtAngleToInches(-45,.5,3);
       while(opModeIsActive())
       {
           robot.moveForTeleOp();
       }
    }

}
