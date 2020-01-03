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

@TeleOp

public class XY_to_ang2 extends LinearOpMode {

    boolean debug = false; //Duh!

    //region TensorFlow

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;

    private static final String VUFORIA_KEY =
            "Ad6cSm3/////AAABmRkDMfGtWktbjulxwWmgzxl9TiuwUBtfA9n1VM546drOcSfM+JxvMxvI1WrLSLNdapOtOebE6n3BkjTjyj+sTXHoEyyJW/lPPmlX5Ar2AjeYpTW/WZM/lzG8qDPsm0tquhEj3BUisA5GRttyGXffPwfKJZNPy3WDqnPxyY/U2v+jQNfZjsWqNvUfp3a3klhVPYd25N5dliMihK3WogqNQnZM9bwJc1wRT0zcczYBJJrhpws9A5H2FpOZD6Ov7GqT+rJdKrU6bh+smoueINDFeaFuYQVMEeo7VOLgkzOeRDpfFmVOVeJrmUv+mwnxfFthAY5v90e4kgekG5OYzRQDS2ta0dbUpG6GoJMoZU2vASSa";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod; //TensorFlow Object Detection engine.

    //endregion
    //region IMU
    BNO055IMU imu;
    Orientation angles;
    //endregion
    //region Sensor
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    private DistanceSensor sensorColorRange;
    private DigitalChannel digital0;

    //endregion
    // region Motor Def
    private DcMotor motor2; //intake
    private DcMotor motor3; //intake
    private Servo servo0;
    private Servo servo1;
    private RevBlinkinLedDriver blinkin;
    private DcMotor liftMotor;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    // endregion

    @Override
    public void runOpMode() {

        // region init Vuforia and TensorFlow
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }
        // endregion
        // region Get and set Motors and sensors Hardware map
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
        // Set Motors
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // endregion
        //region IMU Start
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //endregion

        int skyPos = 0; // SkyStone Config
        int step = 1; // Current Autonomous Step

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("lift Motor",liftMotor.getCurrentPosition());
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            double blockAnglish =7;


            //region Find Block Location
            // Not working in function call so call here
            if (tfod != null ) {
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
                        //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        //telemetry.addData(String.format(" left,top (%d)", i), "%.03f , %.03f",
                        // recognition.getLeft(), recognition.getTop());
                        //telemetry.addData(String.format(" right,bottom (%d)", i), "%.03f , %.03f",
                        // recognition.getRight(), recognition.getBottom());
                    }
                }
            }
            //endregion

            if (debug == false) {

                if (step == 1) { //************ move robot forward //works!!

                    moveByEncoder(500);

                    step += 1;
                    telemetry.addLine("step 1 Complete");
                    telemetry.update();

                } else if (step == 2) { //************ find position of block and point
                    //skyPos = findPos(findAng()); //NOT WORKING, commented function for testing

                    moveAngNoPID(blockAnglish);

                    step += 1;
                    telemetry.addLine("step 2 Complete");
                    telemetry.update();

                } else if (step == 3) { //************ get block //NOT WORKING


                    telemetry.addLine("step 3 put arm down");
                    telemetry.update();

                    putDown();

                    step += 1;
                    telemetry.addLine("step 3 Complete");
                    telemetry.update();
                } else if (step == 4) { //************ start intake //TESTING

                    telemetry.addLine("step 4 put arm dowm");
                    telemetry.update();

                    startIntake();
                    moveByEncoder(5000);
                    stopIntake();
                    moveByEncoder(-5000);

                    step += 10;
                    telemetry.addLine("step 4 Complete");
                    telemetry.update();
                } else if (step == 5) { //************ get the block and come back //TESTING

                    telemetry.addLine("step 5 get block and come back");
                    telemetry.update();

                    getSky();

                    step += 1;
                    telemetry.addLine("step 5 Complete");
                    telemetry.update();
                    sleep(5000);
                } else if (step == 6) { //************ point to the foundation //WORKS (may be the wrong direction)

                    telemetry.addLine("step 6 Point to foundation");
                    telemetry.update();

                    moveToAng(90);

                    step += 1;
                    sleep(5000);
                    telemetry.addLine("step 6 Complete");
                    telemetry.update();
                } else if (step == 7) { //************ go to foundation // WORKS (may not go the right distance)

                    telemetry.addLine("step 7 goto foundation");
                    telemetry.update();

                    moveByEncoder(300);

                    step += 1;
                    telemetry.addLine("step 7 Complete");
                    telemetry.update();
                    sleep(5000);
                } else if (step == 8) { //************ put out block // WORKS

                    telemetry.addLine("step 8 put block out");
                    telemetry.update();

                    moveBlockInExact();

                    step += 1;
                    telemetry.addLine("step 8 Complete");
                    telemetry.update();
                    sleep(5000);
                } else if (step == 9) { //************ go back

                    telemetry.addLine("step 9 go back");
                    telemetry.update();

                    moveByEncoder(-300); // WORKS (may not go the right distance)

                    step += 1;
                    telemetry.addLine("step 9 Complete");
                    telemetry.update();
                    sleep(5000);
                }


            }

            if (debug == true) {
                if (gamepad1.a) {
                    moveByEncoder(600);
                }
                if (gamepad1.b) {
                    startOuttake();;
                }
                if (gamepad1.y) {
                    putDown();
                    startIntake();
                }
                if (gamepad1.x) {
                    putUp();
                    stopIntake();
                } else {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    if (sensorDistance.getDistance(DistanceUnit.CM) < 6){
                        stopIntake();
                    }
                }
            }

            telemetry.update();

        } //while opModeIsActive end

        if (tfod != null) {
            tfod.shutdown();
        }
    } //runOpMode end

    //------------------My Methods------------------//

    /********************************
     * Move intake down
     ********************************/
    void putDown() {

        while(liftMotor.getCurrentPosition() < 1000){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()+50);
            liftMotor.setPower(0.5);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("lift Motor",liftMotor.getCurrentPosition());
            sleep(100);
        }
        liftMotor.setPower(0);
    }

    /********************************
     * Move intake Up
     ********************************/
    void putUp() {
        while(liftMotor.getCurrentPosition() > 60){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()-50);
            liftMotor.setPower(0.5);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("lift Motor",liftMotor.getCurrentPosition());
            sleep(100);
        }
        liftMotor.setPower(0);
    }

    /********************************
     * Start Intake
     ********************************/
    void startIntake() {
        motor2.setPower(0.5);
        motor3.setPower(0.5);
    }

    /********************************
     * Start Outtake
     ********************************/
    void startOuttake() {
        motor2.setPower(-0.5);
        motor3.setPower(-0.5);
    }

    /********************************
     * Stop Intake
     ********************************/
    void stopIntake() {
        motor2.setPower(0);
        motor3.setPower(0);
    }

    /********************************
     * Move forward using Encoder
     ********************************/
    void moveByEncoder(int EncoderMovement) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int encoderTarget = EncoderMovement + leftMotor.getCurrentPosition();
        double heading = angles.firstAngle;
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
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /********************************
     * Move forward until block collected then move back to starting position
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

        while ( !(sensorDistance.getDistance(DistanceUnit.CM) < 6)) { //need to be less than due to NaN

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            dri = 0.5;

            leftMotor.setPower(dri);
            rightMotor.setPower(dri);

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
     * Move Bot To Joystick Position
     ********************************/
    void moveToJoy() {

        double rot;
        double heading;
        double angleError;
        double target;
        double lastError = 0;
        double Pk = 1.5;
        double Dk = 1.5;

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
     * Move Bot to Angle
     ********************************/
    void moveToAng(double target) {

        double Pk = 1.5;
        double Dk = 0;
        double Ik = 0.1;
        double SIk = 0;
        double rot = 0;
        double angleError = 2;
        double lastError = 2;
        double heading;

        while ((angleError > 1) || (angleError < -1)) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;

            angleError = target - heading;
            if (angleError > 180) {
                angleError = angleError - 360;
            } else if (angleError < -180) {
                angleError = angleError + 360;
            }

            rot = (Pk * angleError / 180) + SIk + (Dk * (angleError - lastError) / 180);
            leftMotor.setPower(-(rot));
            rightMotor.setPower(rot);
            lastError = angleError;
            SIk = SIk + (Ik * angleError / 180);

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /********************************
     * Move Bot by Angle
     ********************************/
    void moveAngNoPID(double target) {

        double rot = 0.4;
        double angleError = 2;
        double lastError = 2;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;

        target = heading + target;

        if (target > 180) {
            target = target - 360;
        } else if (target < -180) {
            target = target + 360;
        }


        while ((angleError > 1) || (angleError < -1)) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;

            angleError = heading - target;

            if (angleError > 180) {
                angleError = angleError - 360;
            } else if (angleError < -180) {
                angleError = angleError + 360;
            }


            if (angleError > 0) {
                rot = -0.3;
            } else {
                rot = 0.3;
            }

            leftMotor.setPower(-(rot));
            rightMotor.setPower(rot);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /********************************
     * returns angle to skystone
     ********************************/
    double findAng() {

        //find ang stuff
        double blockAnglish = 1234.5;

        if (tfod != null) {
            blockAnglish = 1;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                blockAnglish = 2;
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition: updatedRecognitions) {
                    blockAnglish = 3;
                    if (recognition.getLabel() == "Skystone") {
                        blockAnglish = (((recognition.getLeft() + recognition.getRight()) / 2) - 350) / 20;
                        //telemetry.addData("Angleish", blockAnglish);
                    }
                    //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format(" left,top (%d)", i), "%.03f , %.03f",
                    // recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format(" right,bottom (%d)", i), "%.03f , %.03f",
                    // recognition.getRight(), recognition.getBottom());
                }
                //telemetry.update();
            }
        }
        return (blockAnglish);
    }


    /********************************
     * returns angle to skystone
     ********************************/
    double findAngNoPID() {

        //find ang stuff
        double blockAnglish = 1234.5;

        if (tfod != null) {
            blockAnglish = 1;
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                blockAnglish = 2;
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition: updatedRecognitions) {
                    blockAnglish = 3;
                    if (recognition.getLabel() == "Skystone") {
                        blockAnglish = (((recognition.getLeft() + recognition.getRight()) / 2) - 350) / 20;
                        //telemetry.addData("Angleish", blockAnglish);
                    }
                    //telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    //telemetry.addData(String.format(" left,top (%d)", i), "%.03f , %.03f",
                    // recognition.getLeft(), recognition.getTop());
                    //telemetry.addData(String.format(" right,bottom (%d)", i), "%.03f , %.03f",
                    // recognition.getRight(), recognition.getBottom());
                }
                //telemetry.update();
            }
        }
        return (blockAnglish);
    }

    /********************************
     * Point Bot to skystone No PID
     ********************************/
    void pointToSkyNoPID() {

        double rot = 0.4;
        double angleError = findAng();

        while ((angleError > 1) || (angleError < -1)) {

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
     * Point Bot to skystone using PID
     ********************************/
    void pointToSky() {

        double Pk = 1.5;
        double Dk = 0;
        double Ik = 0.1;
        double SIk = 0;
        double rot = 0;

        double angleError = findAng();
        double lastError = 0;

        while (((angleError > 1) || (angleError < -1))) {

            angleError = findAng();
            telemetry.addData("angleError", angleError);

            rot = (Pk * angleError / 180) + SIk + (Dk * (angleError - lastError) / 180);
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

        // Instantiate the Vuforia engine
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