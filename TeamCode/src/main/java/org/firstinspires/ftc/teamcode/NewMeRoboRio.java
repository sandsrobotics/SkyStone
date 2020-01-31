package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous

public class NewMeRoboRio extends LinearOpMode {

    boolean DEBUG = false;          //Duh!
    int NUMBER_BLOCK_TO_PICKUP = 2; //wish I had # define
    boolean RED_SIDE = true;        // Field position

    //region TensorFlow and VUFORIA
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

        int blockCollected = 0; // Track number of blocks picked up
        int step = 1; // Current Autonomous Step


        waitForStart();

        while (opModeIsActive()) {

            if (DEBUG == false){

            }

            if (DEBUG == true) { //use this area to test functions

            }

        } //while opModeIsActive end

    } //runOpMode end
    
    //------------------My Methods------------------//

    /********************************
     * Move intake down
     ********************************/
    void putDown() {

        while(liftMotor.getCurrentPosition() < 1000){
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition()+100);
            liftMotor.setPower(0.5);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("lift Motor",liftMotor.getCurrentPosition());
            sleep(5);
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
        motor2.setPower(1);
        motor3.setPower(1);
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

            if (dri > .5) {
                dri = .5;
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
     * Move forward using Encoder NO PID
     ********************************/
    void moveByEncoderNOPID(int EncoderMovement) {

        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + EncoderMovement);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + EncoderMovement);

        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);

        sleep(2000);

        //while(leftMotor.isBusy() || rightMotor.isBusy()) {
        //telemetry.addData("Right", rightMotor.isBusy());
        //telemetry.addData("Left", leftMotor.isBusy());
        //telemetry.update();
        //}

        //leftMotor.setPower(0);
        //rightMotor.setPower(0);
    }

    /********************************
     * Move backwards using Encoder
     ********************************/
    void moveByEncoderBackwards(int EncoderMovement) {

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

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

        while ((Math.abs(encoderError)) >= 10) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            rot = (Pk * angleError) + (Dk * (angleError - lastError));
            dri = (Dpk * encoderError) + (DDk * (encoderError - LastEncoderError));

            if (dri < -1) {
                dri = -1;
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
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    /********************************
     * Move backwards using Encoder
     ********************************/
    void moveByEncoderBackwardsNoPID(int EncoderMovement) {

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + EncoderMovement);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + EncoderMovement);

        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);

        while(leftMotor.isBusy() || rightMotor.isBusy()) {
            telemetry.addData("position", leftMotor.getCurrentPosition());
            telemetry.update();
        }

        sleep(1000);
        //leftMotor.setPower(0);
        //rightMotor.setPower(0);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
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

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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


        while ((angleError > 5) || (angleError < -5)) {

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
     * Move Bot by Angle
     ********************************/
    void moveToAngNoPID(double target) {

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rot = 0.4;
        double angleError = 2;
        double lastError = 2;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;

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
     * Move Bot by Angle
     ********************************/
    void moveToAngNoPID2(double target) {

        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rot = 0.7;
        double angleError = 2;
        double lastError = 2;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;

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
                rot = -0.7;
            } else {
                rot = 0.7;
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
        double angleError = 2;

        while ((angleError > 1) || (angleError < -1)) {

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

                            angleError  =( ( (recognition.getLeft() + recognition.getRight())/2) - 350 ) / 20;

                            telemetry.addData("Angleish",angleError);
                        }
                    }
                }
                else {
                    angleError  = 0;
                }
            }

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

