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
    void moveMotorForwardSeparateAmount(int[] ticks, double power)
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
    void stopMotors()
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(0);
        }
    }
    void setMotorsToPower(double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(power);
        }
    }
    void setMotorsToSeparatePowers(double[] powers)
    {
        int i = 0;
        for(DcMotor motor: motors)
        {
            motor.setPower(powers[i]);
            i++;
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
    double getCorrectionFromPID(PIDCoefficients PID, double error, double lastError, double bias,  double IntegralRange) // this method takes values from -1 to 1 and returns a value from -1 to 1(except for PID coefficients)
    {
        if(Math.abs(error) <= IntegralRange)
        {
            I += error * PID.i;
            I = Math.max(Math.min(I, 1), -1);
        }

        double D = (error - lastError);
        double output = (PID.p * error) + I +(PID.d * D) + bias;
        return Math.max(Math.min(output, 1), -1);
    }

    ////////////
    //movement//
    ////////////
    void moveForwardInches(float inches, double power)
    {
        moveMotorsForward((int)(ticksPerInchForward * inches), power);
    }
    void turnToAng(double targetAngle, double tolerance, int numOfTimesToStayInTolerance, int maxRuntime)
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
            pow = getCorrectionFromPID(turnPID, (error / 180), (lastError / 180), 0, .1);

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

            numOfTimesRun ++;
            if(numOfTimesRun >= maxRuntime || emergencyStop) break;
        }
        stopMotors();
    }
    void turnWithPower(double power)
    {
        leftTopMotor.setPower(power);
        leftBottomMotor.setPower(power);
        rightTopMotor.setPower(-power);
        rightBottomMotor.setPower(-power);
    }
    void strafeSidewaysWithPower(double power)
    {
        leftTopMotor.setPower(power);
        leftBottomMotor.setPower(-power);
        rightTopMotor.setPower(-power);
        rightBottomMotor.setPower(power);
    }
    void strafeSidewaysTicks(int ticks, double power)
    {
        int[] arr = {ticks, -ticks, -ticks, ticks};
        moveMotorForwardSeparateAmount(arr,0);
    }
    void strafeSidewaysInches(float inches, double power)
    {
        strafeSidewaysTicks((int)(ticksPerInchSideways * inches), power);
    }
    void moveAtAngleWithPower(double angle, double power) //in this method angle should be from -180 to 180
    {
        double[] arr = {power,power,power,power};
        if(angle >= 0 && angle <= 90)
        {
            double processedAngle = (45 - angle)/45;
            arr[1] *= processedAngle;
            arr[2] *= processedAngle;
        }
        else if(angle > 90)
        {
            double processedAngle = (135 - angle)/45;
            arr[0] *= processedAngle;
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
        setMotorsToSeparatePowers(arr);
    }
}