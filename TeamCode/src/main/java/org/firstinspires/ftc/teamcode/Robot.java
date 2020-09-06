package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
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

public class Robot
{
    //objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected DcMotor leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    private BNO055IMU imu;
    private List<DcMotor> motors;
    protected FtcDashboard dashboard;

    //user variables
        //debugs
        protected boolean debug_motors = true;
        protected boolean debug_imu = true;
        protected boolean debug_dashboard = true; // turn this to false during competition
        //other
        protected boolean[] flipMotorDir = {true, true, false, false};
        protected int leftTopMotorNum = 0;
        protected int leftBottomMotorNum = 2;
        protected int rightTopMotorNum = 1;
        protected int rightBottomMotorNum = 3;

    // user dashboard variables
    public static int ticksPerInchForward = 100;
    public static int ticksPerInchSideways = 100;
    public static PIDCoefficients turnPID = new PIDCoefficients(0,0,0);
    public static boolean emergencyStop = false;

    // non-user variables
    protected double I = 0;

    Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initHardware();
        if(debug_motors) testMotors(200,-200);
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

    ////////////////
    //motor config//
    ////////////////
    void resetEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    void setMotorsToCoast()
    {
        for(DcMotor motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    void setMotorsToBrake()
    {
        for(DcMotor motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    void setMotorsToRunWithoutEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    void setMotorsToRunWithEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    void setMotorsToPosition(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    void moveMotorsForward(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    void stopMotors()
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(0);
        }
    }
    void setMotorToPower(double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(power);
        }
    }
    void testMotors(int maxTicks, int minTicks)
    {
        resetEncoders();
        setMotorsToPosition(maxTicks,.5);
        setMotorsToPosition(minTicks,.5);
        setMotorsToPosition(0,.5);
    }

    /////////////
    //telemetry//
    /////////////
    Orientation getAngles()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        if(debug_imu) telemetry.addData("angles: ", angles);
        return angles;
    }
    Velocity getVelocity()
    {
        Velocity velocity = imu.getVelocity();
        if(debug_imu) telemetry.addData("velocity: ", velocity);
        return velocity;
    }
    AngularVelocity getAngularVelocity()
    {
        AngularVelocity angularVelocity = imu.getAngularVelocity();
        if(debug_imu) telemetry.addData("angular velocity: ", angularVelocity);
        return angularVelocity;
    }
    Acceleration getAcceleration()
    {
        Acceleration acceleration = imu.getAcceleration();
        if(debug_imu) telemetry.addData("acceleration", acceleration);
        return acceleration;
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
        return angleError;
    }
    double getCorrectionFromPID(PIDCoefficients PID, double error, double lastError, double bias) // this method takes values from -1 to 1 and returns a value from -1 to 1(except for PID coefficients)
    {
        I += error;
        I = Math.max(Math.min(I, 1), -1);

        double D = (error - lastError);

        double output = (PID.p * error) + (PID.i * I) +(PID.d * D) + bias;
        return Math.max(Math.min(output, 1), -1);
    }

    ////////////
    //movement//
    ////////////
    void moveForwardInches(float inches, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + (int)(inches * ticksPerInchForward));
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
    void turn(double targetAngle, double tolerance, int numOfTimesToStayInTolerance, int maxRuntime)
    {
        I = 0;
        double currentAngle = getAngles().thirdAngle;
        double error = findAngleError(currentAngle, targetAngle);
        double lastError;
        double pow;
        int numOfTimesInTolerance = 0;
        int numOfTimesRun = 0;

        setMotorsToRunWithEncoders();

        while(numOfTimesInTolerance < numOfTimesToStayInTolerance)
        {
            lastError = error;
            currentAngle = getAngles().thirdAngle;
            error = findAngleError(currentAngle, targetAngle);
            pow = getCorrectionFromPID(turnPID, (error / 180), (lastError / 180), 0);

            leftTopMotor.setPower(pow);
            leftBottomMotor.setPower(pow);
            rightTopMotor.setPower(-pow);
            rightBottomMotor.setPower(-pow);

            if(Math.abs(error) < tolerance) numOfTimesInTolerance ++;
            else numOfTimesInTolerance = 0;
            numOfTimesRun ++;
            if(numOfTimesRun >= maxRuntime || emergencyStop) break;
        }
        stopMotors();
    }
}