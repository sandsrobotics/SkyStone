package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous

public class XY_to_ang_LUDICROUS_MODE_OTHERSIDE extends LinearOpMode {

    boolean DEBUG = true;          //Duh!
    int NUMBER_BLOCK_TO_PICKUP = 2; //wish I had # define
    boolean RED_SIDE = true;        // Field position
    double rot;
    double hedding;
    double angleError = 0;
    double target = 0;
    double lastError = 0;
    double Pk = 5;
    double Dk = 5;
    double power;
    double timesRun = 0;
    double leftMotorLastPos = 0;
    // eqauation = pulses per rev / (wheel diamiter * pi)
    double encoderPerInch = 383.6 /(4 * 3.14);

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
    //ColorSensor sensorColor;
   // DistanceSensor sensorDistance;
    //private DistanceSensor sensorColorRange;
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
       // sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        // Set Motors
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
                if(step == 1) {

                    telemetry.addLine("0");
                    step += 1;
                }
                if(step == 2) {

                    telemetry.addLine("1");
                    step += 1;
                }
                if(step == 3) {

                    telemetry.addLine("2");
                    step += 1;
                }
                if (step == 4) {

                    telemetry.addLine("3");
                    step += 1;
                }
                if(step == 5) {

                    liftMotor.setTargetPosition(900);
                    liftMotor.setPower(.3);
                    telemetry.addLine("4");
                    step += 1;
                }
                if(step == 6) {

                    telemetry.addLine("5");
                    step += 1;
                }
                if(step == 7) {

                    telemetry.addLine("6");
                    step += 1;
                }
                if(step == 8) {

                    telemetry.addLine("7");
                    step += 1;
                }
                if(step == 9){

                    telemetry.addLine("8");
                    step += 1;
                }
                if(step == 10){

                    servo0.setPosition(1);
                    servo1.setPosition(0);

                    telemetry.addLine("9");
                    step += 1;
                }
                if(step == 11){


                    telemetry.addLine("11");
                    step += 1;

                }
                if(step == 12){


                    telemetry.addLine("12");
                    step += 1;

                }


            }

            if (DEBUG == true) { //use this area to test functions
                if (gamepad1.a) {
                    /*
                    while(opModeIsActive() && !gamepad1.back){
                        telemetry.addLine("0");
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            telemetry.addLine("1");
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel() == "Skystone") {
                                    // stuff when skystone is there

                                    leftMotor.setPower(0);
                                    rightMotor.setPower(0);
                                    telemetry.addLine("2");

                                } else {
                                    // stuff when skystone is not there

                                    leftMotor.setPower(.1);
                                    rightMotor.setPower(.1);
                                    telemetry.addLine("3");

                                }
                            }
                        }
                        telemetry.update();
                    }

                     */
                    test(.05);

                }
                if (gamepad1.b) {

                }
                if (gamepad1.y) {

                }
                if (gamepad1.x) {

                } else {
                    //leftMotor.setPower(0);
                    //rightMotor.setPower(0);
                    //if (sensorDistance.getDistance(DistanceUnit.CM) < 6){

                    //}
                }
            }

            //telemetry.update();

        } //while opModeIsActive end

        if (tfod != null) {
            tfod.shutdown();
        }
    } //runOpMode end

    //------------------My Methods------------------//

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

        boolean outOfRange = true;
        double currentError;
        int inRange = 0;
        if (Math.abs(findError(target)) < tolerance){

            outOfRange = false;

        }

        while((inRange <= 20) && (outOfRange) && opModeIsActive()){

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
            // telemetry.addData("leftMotor", leftMotor.getPower());
            // telemetry.addData("rightMotor", rightMotor.getPower());
            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }
    void moveWithRamp(double distance, double heading, double minPower, double maxPower, double rampUpFactor,double rampDownFactor, double rampDownPersent){

        rightMotor.setPower(0);
        leftMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        double power = 0;
        double error = 0;
        double encoderTicks = distance * encoderPerInch;
        // leftMotorLastPos = leftMotor.getCurrentPosition();
        double distanceLeft;

        leftMotor.setTargetPosition((int) encoderTicks + leftMotor.getCurrentPosition());
        rightMotor.setTargetPosition((int) encoderTicks + rightMotor.getCurrentPosition() );

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while((leftMotor.isBusy() && rightMotor.isBusy()) && opModeIsActive()) {

            distanceLeft = encoderTicks - leftMotor.getCurrentPosition();

            if (Math.abs(distanceLeft) > (Math.abs(encoderTicks) * rampDownPersent)) {

                power += rampUpFactor;
                power = Math.min(power, maxPower);

            } else if (Math.abs(distanceLeft) < (Math.abs(encoderTicks) * rampDownPersent)) {

                power -= rampDownFactor;
                power = Math.max(power, minPower);

            }

            if (distance > 0) {

                error = findError(heading) / 45;

            } else {

                error = -(findError(heading) / 45);

            }


            rightMotor.setPower(power + error);
            leftMotor.setPower(power - error);

        }


        rightMotor.setPower(0);
        leftMotor.setPower(0);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

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

    void test(double speed){

        boolean skyStone = false;



        leftMotor.setPower(speed);
        rightMotor.setPower(speed);

        while(!skyStone && !gamepad1.back){

            telemetry.addLine("0");
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                telemetry.addLine("1");
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel() == "Skystone") {
                        // stuff when skystone is there

                        telemetry.addLine("2");
                        skyStone = true;

                    } else {
                        // stuff when skystone is not there

                        telemetry.addLine("3");

                    }
                }
            }

            telemetry.update();

        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }



}// end of class


