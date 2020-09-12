package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
import org.firstinspires.ftc.teamcode.holo.MotorConfig;

import java.util.Arrays;
import java.util.List;

@Config
public class Robot
{
    MotorConfig motorConfig;

    //objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected DcMotor leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    private BNO055IMU imu;
    private List<DcMotor> motors;
    protected FtcDashboard dashboard;

    //user variables
        //debugs
        protected boolean debug_telemetry = true;
        protected boolean debug_dashboard = true; // turn this to false during competition

        protected boolean debug_methods = true;
        protected boolean debug_imu = true;
        protected boolean debug_motors = true;
        protected boolean test_motors = false;
        //other
        protected boolean[] flipMotorDir = {true, true, false, false};
        protected int leftTopMotorNum = 0;
        protected int leftBottomMotorNum = 2;
        protected int rightTopMotorNum = 1;
        protected int rightBottomMotorNum = 3;

    // user dashboard variables
    public static double ticksPerInchForward = (383.6 / (3.73 * Math.PI)) * 2;
    public static double ticksPerInchSideways = 191.8;
    public static PIDCoefficients turnPID = new PIDCoefficients(.02,0,0);
    public static boolean emergencyStop = false;

    // non-user variables
    protected double I = 0;
    TelemetryPacket packet = null;

    Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        motorConfig = new MotorConfig(motors);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initHardware();
        if(test_motors) testMotors(200,-200);
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

    void testMotors(int maxTicks, int minTicks)
    {
        motorConfig.resetEncoders();
        motorConfig.setMotorsToPosition(maxTicks,.5);
        motorConfig.setMotorsToPosition(minTicks,.5);
        motorConfig.setMotorsToPosition(0,.5);
    }

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
    /*
    double getCorrectionFromPID(PIDCoefficients PID, double error, double lastError, double bias,  double IntegralRange) // this method takes values from -1 to 1 and returns a value from -1 to 1(except for PID coefficients)
    {
        if(Math.abs(error) <= IntegralRange)
        {
            I += error * PID.i;
            I = Math.max(Math.min(I, 1), -1);
        }

        double D = (error - lastError);
        double output = (PID.p * error) + I + (PID.d * D) + bias;
        if(debug_imu) telemetry.addData("correction power uncapped", output);
        return Math.max(Math.min(output, 1), -1);
    }
     */
    double[] powerForMoveAtAngle(double angle, double basePower)
    {
        double[] arr = {basePower, basePower, basePower, basePower};
        double sidewaysMultiplier = ticksPerInchSideways/ticksPerInchForward;

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

    ////////////
    //movement//
    ////////////
    void moveForwardInches(float inches, double power)
    {
        motorConfig.moveMotorsForward((int)(ticksPerInchForward * inches), power);
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

            telemetry.update();

            numOfTimesRun ++;
            if(numOfTimesRun > maxRuntime || emergencyStop) break;
        }
        stopMotors();
    }
     */
    void turnToAngleSimple(double targetAngle, double tolerance, double proportional, double numberOfTimesToStayInTolerance, double maxRuntime)
    {
        targetAngle *= .5;
        tolerance *= .5;
        proportional *= .5;
        double currentAngle = getAngles().thirdAngle;
        double error = findAngleError(currentAngle, targetAngle);

        if(Math.abs(error) > tolerance)
        {
            int numberOfTimesInTolerance = 0;
            int numberOfTimesRun = 0;
            //I = 0;
            motorConfig.setMotorsToRunWithEncoders();
            motorConfig.setMotorsToBrake();

            while(numberOfTimesInTolerance < numberOfTimesToStayInTolerance)
            {
                startTelemetry();
                currentAngle = getAngles().thirdAngle;
                error = findAngleError(currentAngle, targetAngle);
                turnWithPower(error * turnPID.p);

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

                if(emergencyStop || numberOfTimesRun > maxRuntime || numberOfTimesInTolerance >= numberOfTimesToStayInTolerance) break;

                if(debug_methods)
                {
                    if(debug_telemetry)
                    {
                        telemetry.addData("current power: ", error * proportional);
                        telemetry.addData("number of times run: ", numberOfTimesRun);
                        telemetry.addData("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                    if(debug_dashboard)
                    {
                        packet.put("current power: ", error * proportional);
                        packet.put("number of times run: ", numberOfTimesRun);
                        packet.put("number of times in tolerance: ", numberOfTimesInTolerance);
                    }
                }
                updateTelemetry();
                sendTelemetry();
            }
            motorConfig.stopMotors();
        }
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
        motorConfig.moveMotorForwardSeparateAmount(arr,0);
    }
    void strafeSidewaysInches(float inches, double power)
    {
        strafeSidewaysTicks((int)(ticksPerInchSideways * inches), power);
    }

    void moveAtAngleWithPower(double angle, double power) //in this method angle should be from -180 to 180
    {
        motorConfig.setMotorsToSeparatePowersArray(powerForMoveAtAngle(angle,power));
    }
    void moveAtAngleToInches(double angle, double power, float inches)
    {
        double[] arr = powerForMoveAtAngle(angle, power);
        double forwardAmount = Math.abs(Math.abs(angle) - 90)/90;
        double sideWaysAmount = 1 - forwardAmount;
        int totalTicks = (int)((inches*ticksPerInchForward*forwardAmount) + (inches*ticksPerInchSideways*sideWaysAmount));

        int i = 0;
        for(DcMotor motor:motors)
        {
            motor.setTargetPosition((int)(totalTicks * arr[i]));
            arr[i] = Math.abs(arr[i]);
            i++;
        }

        motorConfig.setMotorsToSeparatePowersArray(arr);
        motorConfig.setMotorsToRunToPosition();
    }
    void moveForTeleOp(Gamepad gamepad1)
    {
        leftTopMotor.setPower((-gamepad1.left_stick_y) + gamepad1.left_stick_x + gamepad1.right_stick_x);
        leftBottomMotor.setPower((-gamepad1.left_stick_y) - gamepad1.left_stick_x + gamepad1.right_stick_x);
        rightTopMotor.setPower((-gamepad1.left_stick_y) - gamepad1.left_stick_x - gamepad1.right_stick_x);
        rightBottomMotor.setPower((-gamepad1.left_stick_y) + gamepad1.left_stick_x - gamepad1.right_stick_x);

    }
    void headlessMoveForTeleOp(Gamepad gamepad1, double offset)
    {
        Orientation angles = getAngles();
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
            if(X > 0) motorConfig.setMotorsToSeparatePowersArray(powerForMoveAtAngle(90 - angles.thirdAngle + offset, power));
        }
    }
}