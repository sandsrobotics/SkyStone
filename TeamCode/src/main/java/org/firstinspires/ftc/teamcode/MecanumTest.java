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
       while(opModeIsActive())
       {
           double[] joystick = {gamepad1.left_stick_x, gamepad1.left_stick_y};
           double power;
           if(Math.abs(joystick[0]) > Math.abs(joystick[1])) power = joystick[0];
           else power = joystick[1];
           if(power > 0)
           {
               double angle = (Math.atan2(joystick[1], joystick[0])) * 57;
               robot.moveAtAngleWithPower(angle, power);
           }

       }
    }

}
