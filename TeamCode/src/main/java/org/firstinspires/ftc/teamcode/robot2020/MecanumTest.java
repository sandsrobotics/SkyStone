package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "test2020")
public class MecanumTest extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode()
    {
       robot = new Robot(hardwareMap, telemetry);
       waitForStart();

       robot.movement.moveForwardInches(7,.5);
       robot.movement.turnToAngleSimple(45,1, .011, 50, 1000);
       while(opModeIsActive())
       {
           robot.startTelemetry();
           robot.movement.moveForTeleOp(gamepad1);
           robot.updateTelemetry();
           robot.sendTelemetry();
       }
    }

}
