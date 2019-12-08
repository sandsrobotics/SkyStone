package org.firstinspires.ftc.teamcode; //    this is telling the robot what data is being used

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //importation of data about the robot's hardware
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp // this makes the robot controlled by a controller

public class TeamCodeMMCAqqqqq extends LinearOpMode { // addition of the hardware's software

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private Servo stick;

    private DcMotor intake;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "left-front");
        rightFront = hardwareMap.get(DcMotor.class, "right-front");
        leftRear = hardwareMap.get(DcMotor.class, "left-rear");
        rightRear = hardwareMap.get(DcMotor.class, "right-rear");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stick = hardwareMap.get(Servo.class, "stick");


        waitForStart();

        boolean slowMode = false;
        float x;
        double y;
        float rotate;
        double drive;


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            telemetry.addData("Left Motor Power", leftFront.getPower());
            telemetry.addData("Right Motor Power", rightFront.getPower());
            telemetry.addData("intake power", intake.getPower());
            telemetry.addData("stick", stick.getPosition());
            telemetry.addData("Status", "Running");

            telemetry.update();

            // controler 1

            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            if (gamepad1.a) {
                slowMode = false;
            }
            else if (gamepad1.b){
                slowMode = true;
            }

            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
            drive = -gamepad1.right_stick_y;

            if (slowMode == false) {
                leftFront.setPower(Math.min(Math.max(y + x + drive + rotate, -1), 1));
                rightFront.setPower(Math.min(Math.max((y - x) + (drive - rotate), -1), 1));
                leftRear.setPower(Math.min(Math.max((y - x) + drive + rotate, -1), 1));
                rightRear.setPower(Math.min(Math.max(y + x + (drive - rotate), -1), 1));
            }
            else if (slowMode == true) {
                leftFront.setPower(Math.min(Math.max(y + x + drive + rotate, -1), 1)/2);
                rightFront.setPower(Math.min(Math.max((y - x) + (drive - rotate), -1), 1)/2);
                leftRear.setPower(Math.min(Math.max((y - x) + drive + rotate, -1), 1)/2);
                rightRear.setPower(Math.min(Math.max(y + x + (drive - rotate), -1), 1)/2);
            }
            if (gamepad1.x) {
                intake.setPower(1);
            }
            else if (gamepad1.y) {
                intake.setPower(-1);
            }
            else{
                intake.setPower(0);
            }


            if (gamepad1.right_bumper) {
                stick.setPosition(0);
            }
            else if (gamepad1.left_bumper) {
                stick.setPosition(.5);
            }
        }
    }
}