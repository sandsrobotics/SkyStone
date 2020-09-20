package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "test turn functions output v1.0")
public class Test extends LinearOpMode
{
    Robot robot;

    public static double targetAngle = 45;
    public static double distanceForward = 1;
    public static double distanceSideways = 4;
    public static double moveAngle = 45;
    public static double moveDistance = 3;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

        robot.motorConfig.setMotorsToRunWithEncoders();
        robot.motorConfig.resetEncoders();
        robot.motorConfig.setMotorsToBrake();

        robot.debug_methods = false;
        robot.debug_imu = false;
        robot.debug_motors = false;

        waitForStart();

        robot.startTelemetry();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                robot.movement.turnToAngleSimple(targetAngle,.5,50,1000);
            }
            if(gamepad1.b)
            {
                robot.movement.moveForwardInches(distanceForward,.5);
                robot.motorConfig.setMotorsToRunWithEncoders();
            }
            if(gamepad1.x)
            {
                robot.movement.strafeSidewaysInches(distanceSideways,.5);
                robot.motorConfig.setMotorsToRunWithEncoders();
            }
            if(gamepad1.y)
            {
                robot.movement.moveAtAngleToInches(moveAngle,.5,moveDistance);
                robot.motorConfig.setMotorsToRunWithEncoders();
            }
            robot.movement.moveForTeleOp(gamepad1);
            robot.sendTelemetry();
        }
    }
}
