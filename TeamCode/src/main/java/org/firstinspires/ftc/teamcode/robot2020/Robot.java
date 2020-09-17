package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Arrays;
import java.util.List;

@Config
public class Robot
{
    /////////////
    //user data//
    /////////////
    //debug
    protected boolean debug_telemetry = true;
    protected boolean debug_dashboard = true; // turn this to false during competition

    protected boolean debug_methods = true;
    protected boolean debug_imu = true;
    protected boolean debug_motors = true;
    protected boolean test_motors = false;

    //user dashboard variables
    public static boolean emergencyStop = false;

    //other classes
    public MotorConfig motorConfig;
    public Movement movement;
    public Vision vision;

    //objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    private BNO055IMU imu;
    protected List<DcMotor> motors;
    protected FtcDashboard dashboard;

    //other
    protected double I = 0;
    TelemetryPacket packet;

    Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        motorConfig = new MotorConfig(this);
        movement = new Movement(this);
        vision = new Vision(this);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initHardware();
        motorConfig.initMotors();
        vision.initVuforia();
        if(test_motors) motorConfig.testMotors(200,-200);
    }

    void initHardware()
    {
        ///////
        //imu//
        ///////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /////////////
        //dashboard//
        /////////////
        if(debug_dashboard) dashboard = FtcDashboard.getInstance();
    }

    //------------------My Methods------------------//
    /////////////
    //telemetry//
    /////////////
    Orientation getAngles()
    {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }
    Velocity getVelocity()
    {
        return imu.getVelocity();
    }
    AngularVelocity getAngularVelocity()
    {
        return imu.getAngularVelocity();
    }
    Acceleration getAcceleration()
    {
        return imu.getAcceleration();
    }
    void startTelemetry()
    {
        if(debug_dashboard)
        {
            packet = new TelemetryPacket();
        }
    }
    void updateTelemetry()
    {
            if(debug_imu) {
                if(debug_telemetry)
                {
                    telemetry.addData("angles: ", getAngles());
                    telemetry.addData("rotation: ", getAngles().thirdAngle);
                    telemetry.addData("velocity: ", getVelocity());
                    telemetry.addData("angular velocity: ", getAngularVelocity());
                    telemetry.addData("acceleration", getAcceleration());
                }
                if(debug_dashboard)
                {
                    packet.put("angles: ", getAngles());
                    packet.put("rotation: ", getAngles().thirdAngle);
                    packet.put("velocity: ", getVelocity());
                    packet.put("angular velocity: ", getAngularVelocity());
                    packet.put("acceleration", getAcceleration());
                }
            }
            if(debug_motors)
            {
                if(debug_telemetry)
                {
                telemetry.addData("motor powers: ", motorConfig.getMotorPowers());
                telemetry.addData("motor positions:", motorConfig.getMotorPositions());
                }
                if(debug_dashboard)
                {
                    packet.put("motor powers: ", motorConfig.getMotorPowers());
                    packet.put("motor positions:", motorConfig.getMotorPositions());
                }
            }


    }
    void addTelemetryDouble(String cap, double val)
    {
        if(debug_dashboard) packet.put(cap,val);
        if(debug_telemetry) telemetry.addData(cap,val);
    }
    void addTelemetryString(String cap, String val)
    {
        if(debug_dashboard) packet.put(cap, val);
        if(debug_telemetry) telemetry.addData(cap, val);
    }
    void sendTelemetry()
    {
        if(debug_motors || debug_methods || debug_imu)
        {
            if(debug_dashboard) dashboard.sendTelemetryPacket(packet);
            if(debug_telemetry) telemetry.update();
        }
    }
    ////////////////
    //calculations//
    ////////////////
    double findAngleError(double currentAngle, double targetAngle)
    {
        if (targetAngle > 180) {
            targetAngle = targetAngle - 360;
        } else if (targetAngle < -180) {
            targetAngle = targetAngle + 360;
        }
        double angleError = currentAngle - targetAngle;
        if (angleError > 180) {
            angleError = angleError - 360;
        } else if (angleError < -180) {
            angleError = angleError + 360;
        }
        if(debug_methods)
        {
            if(debug_telemetry)telemetry.addData("angle error: ", angleError);
            if(debug_dashboard)packet.put("angle error: ", angleError);
        }
        return angleError;
    }

    double getCorrectionFromPID(double error, double lastError, double bias,  double IntegralRange) // this method takes values from -180 to 180 and returns a value from -1 to 1
    {
        if(Math.abs(error) <= IntegralRange)
        {
            I += error * Movement.turnPID.i;
            I = Math.max(Math.min(I, 1), -1);
        }

        double D = (error - lastError);
        double output = (Movement.turnPID.p * error) + I + (Movement.turnPID.d * D) + bias;
        if(debug_methods)
        {
            if(debug_telemetry) telemetry.addData("correction power uncapped", output);
            if(debug_dashboard) packet.put("correction power uncapped", output);
        }
        return Math.max(Math.min(output, 1), -1);
    }

    double[] powerForMoveAtAngle(double angle, double basePower)
    {
        double[] arr = {basePower, basePower, basePower, basePower};
        double sidewaysMultiplier = Movement.ticksPerInchSideways/Movement.ticksPerInchForward;

        if(angle >= 0 && angle <= 90)
        {
            double processedAngle = (45 - angle)/45;
            arr[1] *= processedAngle;
            arr[2] *= processedAngle;
        }
        else if(angle > 90)
        {
            double processedAngle = (135 - angle)/45;
            arr[0] *= processedAngle * sidewaysMultiplier;
            arr[1] *= -1;
            arr[2] *= -1;
            arr[3] *= processedAngle;
        }
        else if(angle >= -90 && angle < 0)
        {
            double processedAngle = (angle + 45)/45;
            arr[0] *= processedAngle;
            arr[3] *= processedAngle;
        }
        else
        {
            double processingAngle = (angle + 135)/45;
            arr[0] *= -1;
            arr[1] *= processingAngle;
            arr[2] *= processingAngle;
            arr[3] *=-1;
        }
        if(debug_methods)
        {
            if(debug_telemetry)
            {
                telemetry.addData("moving angle: ", angle);
                telemetry.addData("power for moving at angle: ", arr);
            }
            if(debug_dashboard)
            {
                packet.put("moving angle: ", angle);
                packet.put("power for moving at angle: ", arr);
            }
        }
        return arr;
    }
}