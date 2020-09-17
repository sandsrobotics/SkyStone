package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "test turn functions")
public class Test extends LinearOpMode
{
    Robot robot;

    public static double targetAngle = 45;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap, telemetry);
        double lastError;
        double error;

        waitForStart();

        error = robot.findAngleError(robot.getAngles().thirdAngle, targetAngle);
        robot.I = 0;


        while(opModeIsActive())
        {
            lastError = error;
            error = robot.findAngleError(robot.getAngles().thirdAngle, targetAngle);
            robot.startTelemetry();
            robot.updateTelemetry();
            robot.addTelemetryDouble("angle error: ", error);
            robot.addTelemetryDouble("last angle error: ", lastError);
            robot.addTelemetryDouble("PID calculated power: ",robot.getCorrectionFromPID(error, lastError, 0, .2));
            robot.addTelemetryDouble("P calculated power: ", error * Robot.turnPID.p);
            robot.sendTelemetry();
        }
    }
}
