package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "test2020")
public class MecanumTest extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode()
    {
       robot = new Robot(hardwareMap, telemetry);
       waitForStart();

       robot.moveForwardInches(7,.5);
       robot.turnToAngleSimple(45,1, .011, 50, 1000);
       while(opModeIsActive())
       {
           robot.startTelemetry();
           robot.moveForTeleOp(gamepad1);
           robot.updateTelemetry();
           robot.sendTelemetry();
       }
    }

}
