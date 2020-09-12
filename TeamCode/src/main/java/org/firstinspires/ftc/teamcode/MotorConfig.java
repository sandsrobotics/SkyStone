package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.List;

public class MotorConfig
{
    List<DcMotor> motors;
    public MotorConfig(List<DcMotor> motors)
    {
        this.motors = motors;
    }
    public void resetEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void setMotorsToCoast()
    {
        for(DcMotor motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void setMotorsToBrake()
    {
        for(DcMotor motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void setMotorsToRunWithoutEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void setMotorsToRunWithEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setMotorsToRunToPosition()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setMotorsToPosition(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorsForward(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorForwardSeparateAmount(int[] ticks, double power)
    {
        int i = 0;
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(ticks[i]);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    public void stopMotors()
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(0);
        }
    }
    public void setMotorsToPower(double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(power);
        }
    }
    public void setMotorsToSeparatePowersArray(double[] powers)
    {
        int i = 0;
        for(DcMotor motor: motors)
        {
            motor.setPower(powers[i]);
            i++;
        }
    }
    public double[] getMotorPowers()
    {
        double[] arr = new double[4];
        int i = 0;
        for(DcMotor motor:motors)
        {
            arr[i] = motor.getPower();
            i++;
        }
        return arr;
    }
    public int[] getMotorPositions()
    {
        int[] arr = new int[4];
        int i = 0;
        for(DcMotor motor:motors)
        {
            arr[i] = motor.getCurrentPosition();
            i++;
        }
        return arr;
    }
}
