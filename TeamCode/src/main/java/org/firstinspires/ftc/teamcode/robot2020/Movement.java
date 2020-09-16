package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Movement
{
    Robot robot;
   Movement(Robot robot)
   {
       this.robot = robot;
   }
    void moveForwardInches(float inches, double power)
    {
        robot.motorConfig.moveMotorsForward((int)(Robot.ticksPerInchForward * inches), power);
    }
    /*
    void turnToAngPID(double targetAngle, double tolerance, int numOfTimesToStayInTolerance, int maxRuntime)
    {
        I = 0;
        double currentAngle = getAngles().thirdAngle;
        double error = findAngleError(currentAngle, targetAngle);
        double lastError;
        double pow;
        int numOfTimesInTolerance = 0;
        int numOfTimesRun = 0;

        setMotorsToRunWithEncoders();
        setMotorsToBrake();

        while(numOfTimesInTolerance < numOfTimesToStayInTolerance)
        {
            lastError = error;
            currentAngle = getAngles().thirdAngle;
            error = findAngleError(currentAngle, targetAngle);
            pow = getCorrectionFromPID(turnPID, error, lastError, 0, .1);

            if(Math.abs(error) < tolerance)
            {
                numOfTimesInTolerance ++;
                I = 0;
                pow = 0;
            }
            else numOfTimesInTolerance = 0;

            leftTopMotor.setPower(pow);
            leftBottomMotor.setPower(pow);
            rightTopMotor.setPower(-pow);
            rightBottomMotor.setPower(-pow);

            telemetry.update();

            numOfTimesRun ++;
            if(numOfTimesRun > maxRuntime || emergencyStop) break;
        }
        stopMotors();
    }
     */
    void turnToAngleSimple(double targetAngle, double tolerance, double numberOfTimesToStayInTolerance, double maxRuntime)
    {
        double currentAngle = robot.getAngles().thirdAngle;
        double error = robot.findAngleError(currentAngle, targetAngle);

        if(Math.abs(error) > tolerance)
        {
            int numberOfTimesInTolerance = 0;
            int numberOfTimesRun = 0;
            //I = 0;
            robot.motorConfig.setMotorsToRunWithEncoders();
            robot.motorConfig.setMotorsToBrake();

            while(numberOfTimesInTolerance < numberOfTimesToStayInTolerance)
            {
                robot.startTelemetry();
                currentAngle = robot.getAngles().thirdAngle;
                error = robot.findAngleError(currentAngle, targetAngle);
                turnWithPower(error * Robot.turnPID.p);

                if(Math.abs(error) < tolerance)
                {
                    //I = 0;
                    numberOfTimesInTolerance++;
                }
                else {
                    numberOfTimesInTolerance = 0;
                    //if(error > 0) I += .001;

                }
                numberOfTimesRun++;

                if(Robot.emergencyStop || numberOfTimesRun > maxRuntime || numberOfTimesInTolerance >= numberOfTimesToStayInTolerance) break;

                if(robot.debug_methods)
                {
                    if(robot.debug_telemetry)
                    {
                        robot.telemetry.addData("current power: ", error * Robot.turnPID.p);
                        robot.telemetry.addData("number of times run: ", numberOfTimesRun);
                        robot.telemetry.addData("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                    if(robot.debug_dashboard)
                    {
                        robot.packet.put("current power: ", error * Robot.turnPID.p);
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
        robot.leftTopMotor.setPower(power);
        robot.leftBottomMotor.setPower(power);
        robot.rightTopMotor.setPower(-power);
        robot.rightBottomMotor.setPower(-power);
    }
    void strafeSidewaysWithPower(double power)
    {
        robot.leftTopMotor.setPower(power);
        robot.leftBottomMotor.setPower(-power);
        robot.rightTopMotor.setPower(-power);
        robot.rightBottomMotor.setPower(power);
    }
    void strafeSidewaysTicks(int ticks, double power)
    {
        int[] arr = {ticks, -ticks, -ticks, ticks};
        robot.motorConfig.moveMotorForwardSeparateAmount(arr,0);
    }
    void strafeSidewaysInches(float inches, double power)
    {
        strafeSidewaysTicks((int)(Robot.ticksPerInchSideways * inches), power);
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
        int totalTicks = (int)((inches*Robot.ticksPerInchForward*forwardAmount) + (inches*Robot.ticksPerInchSideways*sideWaysAmount));

        int i = 0;
        for(DcMotor motor:robot.motors)
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
        robot.leftTopMotor.setPower((-gamepad1.left_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x);
        robot.leftBottomMotor.setPower((-gamepad1.left_stick_y) - gamepad1.left_stick_x + gamepad1.right_stick_x);
        robot.rightTopMotor.setPower((-gamepad1.left_stick_y) - gamepad1.left_stick_x - gamepad1.right_stick_x);
        robot.rightBottomMotor.setPower((-gamepad1.left_stick_y) + gamepad1.left_stick_x - gamepad1.right_stick_x);

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
