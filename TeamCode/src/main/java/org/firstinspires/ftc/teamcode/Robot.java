package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.Arrays;
import java.util.List;

public class Robot
{

    DcMotor leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    BNO055IMU imu;
    List<DcMotor> motors;

    //variables
    boolean debug = true;
    boolean[] flipMotorDir = {true, true, false, false};
    int leftTopMotorNum = 0;
    int leftBottomMotorNum = 2;
    int rightTopMotorNum = 1;
    int rightBottomMotorNum = 3;
    int ticksToInches = 100;

    Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initHardware();
        if(debug) testMotors();
    }

    void initHardware()
    {
        //////////
        //motors//
        //////////
        leftTopMotor = hardwareMap.dcMotor.get("motor" + leftTopMotorNum);
        leftBottomMotor = hardwareMap.dcMotor.get("motor" + leftBottomMotorNum);
        rightTopMotor = hardwareMap.dcMotor.get("motor" + rightTopMotorNum);
        rightBottomMotor = hardwareMap.dcMotor.get("motor" + rightBottomMotorNum);
        motors = Arrays.asList(leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor);

        //////////////
        //motor init//
        //////////////
        int i = 0;
        for(DcMotor motor:motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(flipMotorDir[i]) motor.setDirection(DcMotor.Direction.REVERSE);
            i++;
        }

        ////////
        // imu//
        ////////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    void testMotors()
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(100);
        }
        resetEncoders();
    }
    void resetEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void moveForwardInches(float inches, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(inches * ticksToInches));
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    void moveForwardTicks(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}
