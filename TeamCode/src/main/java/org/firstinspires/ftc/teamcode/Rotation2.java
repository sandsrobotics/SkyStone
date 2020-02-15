//////////////////////
//Date: feb 15, 2020//
//tested rotation: //
//works with overshoot//
////////////////////////
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Vuforia

@TeleOp

public class Rotation2 extends LinearOpMode {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "Ad6cSm3/////AAABmRkDMfGtWktbjulxwWmgzxl9TiuwUBtfA9n1VM546drOcSfM+JxvMxvI1WrLSLNdapOtOebE6n3BkjTjyj+sTXHoEyyJW/lPPmlX5Ar2AjeYpTW/WZM/lzG8qDPsm0tquhEj3BUisA5GRttyGXffPwfKJZNPy3WDqnPxyY/U2v+jQNfZjsWqNvUfp3a3klhVPYd25N5dliMihK3WogqNQnZM9bwJc1wRT0zcczYBJJrhpws9A5H2FpOZD6Ov7GqT+rJdKrU6bh+smoueINDFeaFuYQVMEeo7VOLgkzOeRDpfFmVOVeJrmUv+mwnxfFthAY5v90e4kgekG5OYzRQDS2ta0dbUpG6GoJMoZU2vASSa";

    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    // trackables
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    //Motor
    private DcMotor leftMotor;
    private DcMotor rightMotor;


    double rot;
    double hedding;
    double angleError = 0;
    double target = 0;
    double lastError = 0;
    double Pk = 5;
    double Dk = 5;
    Position position;
    double xPos;
    double yPos;
    double zPos;
    double power;
    double timesRun = 0;

    boolean debug = true;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    private DigitalChannel digital0;
    private DcMotor motor2;
    private DcMotor motor3;
    private ColorSensor sensorColorRange;
    private Servo servo0;
    private Servo servo1;
    private RevBlinkinLedDriver blinkin;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor0B;
    private DcMotor liftMotor;

    @Override
    public void runOpMode() {

        digital0 = hardwareMap.digitalChannel.get("digital0");
        liftMotor = hardwareMap.dcMotor.get("motor0B");
        leftMotor = hardwareMap.dcMotor.get("motor0");
        rightMotor = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        servo0 = hardwareMap.servo.get("servo0"); // test
        servo1 = hardwareMap.servo.get("servo1");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters Vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        Vparameters.vuforiaLicenseKey = VUFORIA_KEY;
        Vparameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(Vparameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsSkyStone);
        //*//*//*//*//*//*//*//

        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, Vparameters.cameraDirection);
        }

        targetsSkyStone.activate();
        // end of view

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //      rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
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
            telemetry.addData("current heading", getHeading());
            telemetry.addData("version " , "4.3");
            telemetry.update();

            if (debug == true) {
                if (gamepad1.b) {

                    MoveToAngleWithFakePID(90,1,100,.05 , 2.5);

                }
                else if(gamepad1.a) {

                    //crazyRotation(90,.5,.2);
                    MoveToAngleWithFakePID(90,.5,180,.03 , 2.5);

                }
                else {
                    //leftMotor.setPower(0);
                    //rightMotor.setPower(0);
                }
            }
        }
    }

    void moveToAng(double target, double tolerance) {

        telemetry.update();
        hedding = angles.firstAngle;
        angleError = target - hedding;
        double absulutError =  java. lang. Math. abs(angleError);

        while (absulutError > tolerance) {

            if(hedding > target) {
                rot = (Pk * angleError / 180) + (Dk * (angleError - lastError) / 180);
                leftMotor.setPower(-(rot));
                rightMotor.setPower(rot);
                lastError = angleError;

                telemetry.update();
                hedding = angles.firstAngle;
                angleError = target - hedding;
                absulutError = java.lang.Math.abs(angleError);
            }

            if(hedding < target) {
                rot = (Pk * angleError / 180) + (Dk * (angleError - lastError) / 180);
                leftMotor.setPower(rot);
                rightMotor.setPower(-rot);
                lastError = angleError;

                telemetry.update();
                hedding = angles.firstAngle;
                angleError = target - hedding;
                absulutError = java.lang.Math.abs(angleError);
            }

            telemetry.addData("leftMotor", leftMotor.getPower());
            telemetry.addData("rightMotor", rightMotor.getPower());

            if(gamepad1.back){

                break;

            }
        }
    }
/*
    void crazyRotation(double dgreezz, double tolerancess, double speed) {

        hedding = getHeading();
        //if you want it to reset every time
        //dgreezz += angles.firstAngle;

        if(hedding < dgreezz - tolerancess){
            while(hedding < dgreezz - tolerancess) {



                leftMotor.setPower(-speed);
                rightMotor.setPower(speed);

                if(gamepad1.back){

                    break;

                }
                hedding = getHeading();
            }
        }
        else if (hedding > dgreezz + tolerancess){
            while(hedding > dgreezz + tolerancess) {



                leftMotor.setPower(speed);
                rightMotor.setPower(-speed);

                if(gamepad1.back){

                    break;

                }
                hedding = getHeading();
            }
        }
    }

 */

    double getHeading(){

        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;


    }

    double findError(double target1){

        double hedding1 = getHeading();

        double angleError1 = target1 - hedding1;
        if (angleError1 > 180) {
            angleError1 = angleError1 - 360;
        } else if (angleError1 < -180) {
            angleError1 = angleError1 + 360;
        }

        return angleError1;

    }

    void MoveToAngleWithFakePID (double target, double tolerance , double divitionFactor, double minPower, double minBreakSpeed){

        double currentError;
        int inRange = 0;

        while(inRange <= 40 && opModeIsActive()){

            currentError = findError(target);

            if (Math.abs(currentError) < tolerance){

                inRange ++;
                power = 0;

            }else{

                inRange = 0;
                power = (findError(target) / divitionFactor);

                if (currentError > 0){

                    power += minPower;

                }else{

                    power -= minPower;

                }
            }

            leftMotor.setPower(-power);
            rightMotor.setPower(power);

            if(gamepad1.back){

                break;

            }

            //telemetry.addData("current heading", getHeading());
            telemetry.addData("angleError", currentError);
            telemetry.addData("inRange", inRange);
            telemetry.addData("time loop run", timesRun ++);
            telemetry.addData("min power", minPower);
            telemetry.addData("RightMotor", leftMotor.getPower());
            telemetry.addData("LeftMotor", rightMotor.getPower());
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

}// end of class

