package org.firstinspires.ftc.teamcode; //    this is telling the robot what data is being used

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //importation of data about the robot's hardware
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp // this makes the robot controlled by a controller

public class TeamCodeMMCB extends LinearOpMode { // addition of the hardware's software

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor slideOut;
    private Servo Servo;

    @Override

    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        slideOut = hardwareMap.get(DcMotor.class, "slideOut");
        Servo = hardwareMap.servo.get("left_handAsServo");

        waitForStart();

        double tgtPower2 = 0;
        double tgtPower = 0;
        double NewIdea = 1;
        double ServoPow = 0;

        while (opModeIsActive()) {

            telemetry.addData("Left Motor Power", leftFront.getPower());
            telemetry.addData("Right Motor Power qqqqqq", rightFront.getPower());
            telemetry.addData("Servo pos", ServoPow);
            telemetry.addData("Status", "Running");

            telemetry.update();

            // controler 1

            //driving

            if (gamepad1.a) {
                NewIdea = 1;
            }
            if (gamepad1.b) {
                NewIdea = .5;
            }
            tgtPower = this.gamepad2.left_stick_y;
            leftFront.setPower(tgtPower * NewIdea);
            tgtPower2 = -this.gamepad2.right_stick_y;
            rightFront.setPower(tgtPower2 * NewIdea);
            // slide out
            if (gamepad1.x) {
                slideOut.setPower(.5);
            }
            else if (gamepad1.y) {
                slideOut.setPower(-.5);
            }
            else {
                slideOut.setPower(0);
            }
            // servo
            if (gamepad1.dpad_down) {
                ServoPow = ServoPow + 1;
            }
            else if (gamepad1.dpad_up) {
                ServoPow = ServoPow - 1;
            }
            Servo.setPosition(ServoPow);

        }
    }
}