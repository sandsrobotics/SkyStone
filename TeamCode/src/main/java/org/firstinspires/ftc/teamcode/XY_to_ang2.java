package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Vuforia
//added for TensorFlow Object Detection

@TeleOp

public class XY_to_ang2 extends LinearOpMode {

    //added for TensorFlow Object Detection
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

    private static final String VUFORIA_KEY =
            "Ad6cSm3/////AAABmRkDMfGtWktbjulxwWmgzxl9TiuwUBtfA9n1VM546drOcSfM+JxvMxvI1WrLSLNdapOtOebE6n3BkjTjyj+sTXHoEyyJW/lPPmlX5Ar2AjeYpTW/WZM/lzG8qDPsm0tquhEj3BUisA5GRttyGXffPwfKJZNPy3WDqnPxyY/U2v+jQNfZjsWqNvUfp3a3klhVPYd25N5dliMihK3WogqNQnZM9bwJc1wRT0zcczYBJJrhpws9A5H2FpOZD6Ov7GqT+rJdKrU6bh+smoueINDFeaFuYQVMEeo7VOLgkzOeRDpfFmVOVeJrmUv+mwnxfFthAY5v90e4kgekG5OYzRQDS2ta0dbUpG6GoJMoZU2vASSa";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    private boolean targetVisible = false;

    //Motor
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    int skyPos;

    double rot;
    double heading;
    double angleError = 0;
    double target = 0;
    double lastError = 0;
    double Pk = 1.5;
    double Dk = 1.5;
    int step = 1;

    double blockAnglish = 0;

    int skyPosTest;

    boolean debug = true;

    BNO055IMU imu;
    Orientation angles;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    Acceleration gravity;

    private DistanceSensor sensorColorRange;
    private DigitalChannel digital0;
    private DcMotor motor2;
    private DcMotor motor3;
    private Servo servo0;
    private Servo servo1;
    private RevBlinkinLedDriver blinkin;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor0B;

    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        digital0 = hardwareMap.digitalChannel.get("digital0");
        motor0B = hardwareMap.dcMotor.get("motor0B");
        leftMotor = hardwareMap.dcMotor.get("motor0");
        rightMotor = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor3 = hardwareMap.dcMotor.get("motor3");
        servo0 = hardwareMap.servo.get("servo0"); // test
        servo1 = hardwareMap.servo.get("servo1");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        int skyPos = 0; // SkyStone Config

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

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

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // double DriveSpeed;
        // double Rotate;
        // double v0;
        // double v1;
        // double maxValue;

        while (opModeIsActive()) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);


            //telemetry.addData("RightMotor", leftMotor.getPower());
            //telemetry.addData("LeftMotor", rightMotor.getPower());
            //telemetry.addData("This is E", angleError);
            //telemetry.addData("This is target", target);

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if(recognition.getLabel()=="Skystone") {
                            blockAnglish =( ( (recognition.getLeft() + recognition.getRight())/2) - 350 ) / 20;
                            telemetry.addData("Angleish",blockAnglish);
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    //telemetry.update();
                }
            }

            debug = false; // switch modes

            if(debug == false) {
                skyPos = 1; // &*&*&*&*&*&*&*&*&*&*&*&*&*&*&*# added for testing
                step = 3;// &*&*&*&*&*&*&*&*&*&*&*&*&*&*&*# added for testing

                if (step == 1){ //************ move robot forward       //works!!
                    moveByEncoder(50);

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if (step == 2){ //************ find position of block      //NOT WORKING, commented function for testing
                    //skyPos = findPos(findAng());
                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if(step == 3){ //************ point to block       //NOT WORKING

                    if (skyPos == 1) {
                        moveToAng(7.5);
                    } else if (skyPos == 2) {
                        pointToSky();
                    } else {
                        pointToSky();
                    }

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if (step == 4){ //************ put down the arm  //TESTING
                    putDown();

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if(step == 5){ //************ get the block and come back //TESTING
                    getSky();

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if(step == 6){ //************ point to the foundation //WORKS (may be the wrong direction)
                    moveToAng(90);

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if(step == 7){ //************ go to foundation // WORKS (may not go the right distance)
                    moveByEncoder(3000);

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if(step == 8){ //************ put out block // WORKS
                    moveBlockInExact();

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }
                else if(step == 9){ //************ go back
                    moveByEncoder(-3000); // WORKS (may not go the right distance)

                    step += 1;
                    telemetry.update();
                    sleep(1000);
                }


            }

            if (debug == true) {
                if (gamepad1.a) {
                    moveByEncoder(600);
                }
                if (gamepad1.b) {
                    pointToSky();
                }
                if (gamepad1.y) {
                    telemetry.addData("findAng()", findAng());
                }
                if(gamepad1.x){
                    while(gamepad1.x) {
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        //telemetry.addData("findAng", findAng());
                        telemetry.addData("heading", angles.firstAngle);
                        //telemetry.update();
                    }
                }
                else {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }
            }

            telemetry.update();

        }//while opModeIsActive end
    }//runOpMode end

    /********************************
     *    Move forward using Encoder
     ********************************/

    void putDown(){

        while (motor0B.getCurrentPosition() > -1000 + 20) {
            motor0B.setTargetPosition(-1000);
            motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (motor0B.getCurrentPosition() > -420) {
                motor0B.setPower(0.5);
            } else if (motor0B.getCurrentPosition() > -1000) {
                motor0B.setPower(0.1);
            }
        }
        motor0B.setPower(0);
    }
    void moveByEncoder(int EncoderMovement) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int encoderTarget = EncoderMovement + leftMotor.getCurrentPosition();
        heading = angles.firstAngle;
        double angleError = angles.firstAngle - heading;
        int startingPos = leftMotor.getCurrentPosition();
        int encoderError = encoderTarget - leftMotor.getCurrentPosition();
        double rot = 1.5; // make drive double, drive pk
        double Pk = 2.5 / 180;
        double Dk = 1.5 / 180;
        double dri = 1.5; // 383.6
        double Dpk = 1;
        double DDk = .001;
        double lastError = angleError;
        int LastEncoderError = 0;

        while (encoderError >= 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            rot = (Pk * angleError) + (Dk * (angleError - lastError));
            dri = (Dpk * encoderError) + (DDk * (encoderError - LastEncoderError));

            if (dri > 1) {
                dri = 1;
            }

            leftMotor.setPower(rot + dri);
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

    /********************************
     *    Move forward until block collected then move back to starting position
     ********************************/
    void getSky() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int encoderTarget = 5000 + leftMotor.getCurrentPosition();
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

        motor2.setPower(0.5);
        motor3.setPower(0.5);

        while (sensorDistance.getDistance(DistanceUnit.CM) > 6) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            rot = (Pk * angleError) + (Dk * (angleError - lastError));
            dri = (Dpk * encoderError) + (DDk * (encoderError - LastEncoderError));

            if (dri > 1) {
                dri = 1;
            }

            leftMotor.setPower(rot + dri);
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

        motor2.setPower(0);
        motor3.setPower(0);

        moveByEncoder(startingPos - (leftMotor.getCurrentPosition()));
    }

    /********************************
     *    Move Bot To Joystick Position
     ********************************/
    void moveToJoy() {
        if ((Math.abs(gamepad1.left_stick_x)) + (Math.abs(gamepad1.left_stick_y)) + (Math.abs(gamepad1.right_stick_y)) > 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;

            target = Math.atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) / Math.PI * 180;

            angleError = target - heading;
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

    /********************************
     *    Move Bot to Angle
     ********************************/
    void moveToAng(double target) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;

        angleError = target - heading;
        if (angleError > 180) {
            angleError = angleError - 360;
        } else if (angleError < -180) {
            angleError = angleError + 360;
        }

        rot = (Pk * angleError / 180) + (Dk * (angleError - lastError) / 180);
        leftMotor.setPower(-(rot));
        rightMotor.setPower(rot);
        lastError = angleError;
    }

    /********************************
     *    returns angle to skystone
     ********************************/
    double findAng() {

        //find ang stuff
        double blockAnglish = 1234.5;

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            sleep(250);

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel()=="Skystone") {
                        blockAnglish =( ( (recognition.getLeft() + recognition.getRight())/2) - 350 ) / 20;
                        telemetry.addData("Angleish",blockAnglish);
                    }
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                }
                //telemetry.update();
            }
        }
        return (blockAnglish);
    }
    /********************************
     *    Point Bot to skystone No PID
     ********************************/
    void pointToSkyNoPID() {

        double angleError = findAng();

        while ((angleError > 1) || (angleError < -1) ) {

            angleError = findAng();
            telemetry.addData("angleError", angleError);

            if (angleError > 0) {
                rot = -0.1;
            } else {
                rot = 0.1;
            }

            leftMotor.setPower((rot));
            rightMotor.setPower(-(rot));
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /********************************
     *    Point Bot to skystone using PID
     ********************************/
    void pointToSky() {

        double Pk = 1.5;
        double Dk = 0;
        double Ik = 0.1;
        double SIk = 0;

        double angleError = findAng();

        while ( ((angleError > 1) || (angleError < -1) ) ) {

            angleError = findAng();
            telemetry.addData("angleError", angleError);

            rot = (Pk * angleError / 180) + SIk + (Dk * (angleError - lastError) / 180 ) ;
            leftMotor.setPower((rot));
            rightMotor.setPower(-(rot));
            lastError = angleError;
            //SIk = SIk + (Ik * angleError / 180);

            if (angleError == 1234.5) {
                angleError = 0;
                telemetry.addLine("Exit Through 1234.5");
            }
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /**
     * Shoots out block
     */
    void moveBlockInExact() {

        while (sensorDistance.getDistance(DistanceUnit.CM) >= 6) {
            motor2.setPower(0.5);
            motor3.setPower(0.5);
        }
        motor2.setPower(0);
        motor3.setPower(0);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

} //end of class