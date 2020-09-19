package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Movement
{
    //////////////////
    //user variables//
    //////////////////
    public static double ticksPerInchForward = (383.6 / (3.73 * Math.PI)) * 2;
    public static double ticksPerInchSideways = 191.8;
    public static PIDCoefficients turnPID = new PIDCoefficients(.02,0,0);

    //other class
    Robot robot;

    Movement(Robot robot)
    {
        this.robot = robot;
    }

    void moveForwardInches(float inches, double power)
    {
        robot.motorConfig.moveMotorsForward((int)(ticksPerInchForward * inches), power);
    }

    void turnToAngPID(double targetAngle, double tolerance, int numOfTimesToStayInTolerance, int maxRuntime)
    {
        double I = 0;
        double currentAngle = robot.getAngles().thirdAngle;
        double error = robot.findAngleError(currentAngle, targetAngle);
        double lastError;
        double pow;
        int numOfTimesInTolerance = 0;
        int numOfTimesRun = 0;

        //robot.motorConfig.setMotorsToRunWithEncoders();
        //robot.motorConfig.setMotorsToBrake();

        while(numOfTimesInTolerance < numOfTimesToStayInTolerance)
        {
            lastError = error;
            currentAngle = robot.getAngles().thirdAngle;
            error = robot.findAngleError(currentAngle, targetAngle);
            pow = robot.getCorrectionFromPID(error, lastError,0,.1);

            if(Math.abs(error) < tolerance)
            {
                numOfTimesInTolerance ++;
                I = 0;
                pow = 0;
            }
            else numOfTimesInTolerance = 0;

            robot.motorConfig.leftTopMotor.setPower(pow);
            robot.motorConfig.leftBottomMotor.setPower(pow);
            robot.motorConfig.rightTopMotor.setPower(-pow);
            robot.motorConfig.rightBottomMotor.setPower(-pow);

            numOfTimesRun ++;
            if(numOfTimesRun > maxRuntime || Robot.emergencyStop) break;
        }
        robot.motorConfig.stopMotors();
    }

    void turnToAngleSimple(double targetAngle, double tolerance, double numberOfTimesToStayInTolerance, double maxRuntime)
    {
        double currentAngle = robot.getAngles().thirdAngle;
        double error = robot.findAngleError(currentAngle, targetAngle);
        robot.sendTelemetry();

        if(Math.abs(error) > tolerance)
        {
            int numberOfTimesInTolerance = 0;
            int numberOfTimesRun = 0;
            //I = 0;
            robot.motorConfig.setMotorsToRunWithEncoders();
            robot.motorConfig.setMotorsToBrake();

            while(numberOfTimesInTolerance < numberOfTimesToStayInTolerance)
            {
                currentAngle = robot.getAngles().thirdAngle;
                error = robot.findAngleError(currentAngle, targetAngle);
                turnWithPower(error * turnPID.p);

                if(Math.abs(error) < tolerance)
                {
                    robot.I = 0;
                    numberOfTimesInTolerance++;
                }
                else {
                    numberOfTimesInTolerance = 0;
                }
                numberOfTimesRun++;

                if(Robot.emergencyStop || numberOfTimesRun > maxRuntime || numberOfTimesInTolerance >= numberOfTimesToStayInTolerance) break;

                if(robot.debug_methods)
                {
                    if(robot.debug_telemetry)
                    {
                        robot.telemetry.addData("current power: ", error * turnPID.p);
                        robot.telemetry.addData("number of times run: ", numberOfTimesRun);
                        robot.telemetry.addData("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                    if(robot.debug_dashboard)
                    {
                        robot.packet.put("current power: ", error * turnPID.p);
                        robot.packet.put("number of times run: ", numberOfTimesRun);
                        robot.packet.put("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                }
                robot.updateTelemetry();
                robot.sendTelemetry();
            }
            robot.motorConfig.stopMotors();
        }
    }
    void turnWithPower(double power)
    {
        robot.motorConfig.leftTopMotor.setPower(power);
        robot.motorConfig.leftBottomMotor.setPower(power);
        robot.motorConfig.rightTopMotor.setPower(-power);
        robot.motorConfig.rightBottomMotor.setPower(-power);
    }
    void strafeSidewaysWithPower(double power)
    {
        robot.motorConfig.leftTopMotor.setPower(power);
        robot.motorConfig.leftBottomMotor.setPower(-power);
        robot.motorConfig.rightTopMotor.setPower(-power);
        robot.motorConfig.rightBottomMotor.setPower(power);
    }
    void strafeSidewaysTicks(int ticks, double power)
    {
        int[] arr = {ticks, -ticks, -ticks, ticks};
        robot.motorConfig.moveMotorForwardSeparateAmount(arr,0);
    }
    void strafeSidewaysInches(float inches, double power)
    {
        strafeSidewaysTicks((int)(ticksPerInchSideways * inches), power);
    }

    void moveAtAngleWithPower(double angle, double power) //in this method angle should be from -180 to 180
    {
        robot.motorConfig.setMotorsToSeparatePowersArray(robot.powerForMoveAtAngle(angle,power));
    }
    void moveAtAngleToInches(double angle, double power, float inches)
    {
        double[] arr = robot.powerForMoveAtAngle(angle, power);
        double forwardAmount = Math.abs(Math.abs(angle) - 90)/90;
        double sideWaysAmount = 1 - forwardAmount;
        int totalTicks = (int)((inches*ticksPerInchForward*forwardAmount) + (inches*ticksPerInchSideways*sideWaysAmount));

        int i = 0;
        for(DcMotor motor: robot.motorConfig.motors)
        {
            motor.setTargetPosition((int)(totalTicks * arr[i]));
            arr[i] = Math.abs(arr[i]);
            i++;
        }

        robot.motorConfig.setMotorsToSeparatePowersArray(arr);
        robot.motorConfig.setMotorsToRunToPosition();
    }
    void moveForTeleOp(Gamepad gamepad1)
    {
        robot.motorConfig.leftTopMotor.setPower((-gamepad1.left_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x);
        robot.motorConfig.leftBottomMotor.setPower((-gamepad1.left_stick_y) - gamepad1.left_stick_x + gamepad1.right_stick_x);
        robot.motorConfig.rightTopMotor.setPower((-gamepad1.left_stick_y) - gamepad1.left_stick_x - gamepad1.right_stick_x);
        robot.motorConfig.rightBottomMotor.setPower((-gamepad1.left_stick_y) + gamepad1.left_stick_x - gamepad1.right_stick_x);

    }
    void headlessMoveForTeleOp(Gamepad gamepad1, double offset)
    {
        Orientation angles = robot.getAngles();
        double X = gamepad1.left_stick_x;
        double Y = gamepad1.left_stick_y;
        double power;
        if(Math.abs(X) > Math.abs(Y)) power = X;
        else power = Y;
        if(X != 0 && Y != 0)
        {
            double ang = Math.atan2(Y, X) * 57;
        }
        else if(X != 0)
        {
            if(X > 0) robot.motorConfig.setMotorsToSeparatePowersArray(robot.powerForMoveAtAngle(90 - angles.thirdAngle + offset, power));
        }
    }
}
