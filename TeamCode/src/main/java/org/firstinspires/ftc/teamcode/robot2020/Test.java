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

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        robot.debug_methods = false;
        robot.debug_imu = false;
        robot.debug_motors = false;
        //double lastError;
        double error;

        waitForStart();

        robot.startTelemetry();
        //error = robot.findAngleError(robot.getAngles().thirdAngle, targetAngle);
        //robot.I = 0;

        robot.movement.moveForwardInches(12,.5);

        while(opModeIsActive())
        {
            //lastError = error;
            error = robot.findAngleError(robot.getAngles().thirdAngle, targetAngle);
            //robot.updateTelemetry();
            robot.addTelemetryDouble("current angle: ", robot.getAngles().thirdAngle);
            robot.addTelemetryDouble("angle error: ", error);
            //robot.addTelemetryDouble("last angle error: ", lastError);
            //robot.addTelemetryDouble("PID calculated power: ",robot.getCorrectionFromPID(error, lastError, 0, .2));
            robot.addTelemetryDouble("P calculated power: ", error * Movement.turnPID.p);
            robot.movement.moveForTeleOp(gamepad1);
            robot.sendTelemetry();
        }
    }
}
