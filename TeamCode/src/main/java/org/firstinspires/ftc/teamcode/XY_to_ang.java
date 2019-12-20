package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp

public class XY_to_ang extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    double rot;
    double hedding;
    double angleError = 0;
    double target = 0;
    double lastError = 0;
    double Pk = 1.5;
    double Dk = 1.5;
    Position position;
    double xPos;
    double yPos;
    double zPos;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override

    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        composeTelemetry();
        telemetry.update();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // double DriveSpeed;
        // double Rotate;
        // double v0;
        // double v1;
        // double maxValue;

        while (opModeIsActive()) {

            Position position = imu.getPosition();
            double xPos = position.x;
            double yPos = position.y;
            double zPos = position.z;

            Acceleration acceleration = imu.getAcceleration();
            Velocity velocity = imu.getVelocity();
            position = imu.getPosition();
            xPos = position.x;
            yPos = position.y;
            zPos = position.z;

            telemetry.addData("Xpos", zPos);
            telemetry.addData("Ypos", yPos);
            telemetry.addData("Zpos", xPos);
            telemetry.addData("RightMotor", leftMotor.getPower());
            telemetry.addData("LeftMotor", rightMotor.getPower());
            telemetry.addData("This is E", angleError);
            telemetry.addData("This is target", target);
            telemetry.update();

            if (gamepad1.a) {
                moveByEncoder(5000);
            }
            if (gamepad1.b){
                moveToAng();
            }

            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func < String > () {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func < String > () {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func < String > () {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func < String > () {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func < String > () {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func < String > () {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func < String > () {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel * gravity.xAccel +
                                        gravity.yAccel * gravity.yAccel +
                                        gravity.zAccel * gravity.zAccel));
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    void moveByEncoder(int EncoderMovement) {

        telemetry.update();
        int encoderTarget = EncoderMovement + leftMotor.getCurrentPosition();
        double heading = angles.firstAngle;
        double angleError = angles.firstAngle - heading;
        int startingPos = leftMotor.getCurrentPosition();
        int encoderError = encoderTarget - leftMotor.getCurrentPosition();
        double rot = 1.5; // make drive double, drive pk
        double Pk = 2.5 / 180;
        double Dk = 1.5 / 180;
        double dri = 1.5;
        double Dpk = 1;
        double DDk = .001;
        double lastError = angleError;
        int LastEncoderError = 0;

        while (encoderError >= 0) {

            telemetry.update();

            rot = (Pk  * angleError)   + (Dk  * (angleError   - lastError));
            dri = (Dpk * encoderError) + (DDk * (encoderError - LastEncoderError));

            if (dri > 1) {
                dri = 1;
            }

            leftMotor.setPower (rot + dri);
            rightMotor.setPower(dri - rot);

            encoderError = encoderTarget - leftMotor.getCurrentPosition();
            lastError = angleError;
            angleError = angles.firstAngle - heading;
            telemetry.addData("encoderError", encoderError);
            telemetry.addData("rot", rot);
            telemetry.addData("angleError", angleError);
            telemetry.addData("heading", heading);

            LastEncoderError = encoderError;
        }
    }

    void moveToAng(){
        if ((Math.abs(gamepad1.left_stick_x)) + (Math.abs(gamepad1.left_stick_y)) + (Math.abs(gamepad1.right_stick_y)) > 0) {
            hedding = angles.firstAngle;
            target = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;

            angleError = target - hedding;
            if (angleError > 180) {
                angleError = angleError - 360;
            } else if (angleError < -180) {
                angleError = angleError + 360;
            }

            rot = (Pk * angleError / 180) + (Dk * (angleError - lastError) / 180);
            leftMotor.setPower((-(rot)) + (-((gamepad1.right_stick_y) / 2)));
            rightMotor.setPower((rot) + (-((gamepad1.right_stick_y) / 2)));
            lastError = angleError;
        }

    }

}