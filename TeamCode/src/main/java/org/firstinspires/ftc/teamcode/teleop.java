package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop v9", group = "")
public class teleop extends LinearOpMode {

    private Servo servo0;
    private Servo servo1;
    private DigitalChannel digital0;
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor0B;
    private DcMotor motor2;
    private RevBlinkinLedDriver blinkin;
    private DcMotor motor3;
    private DcMotor park_assist;

    //my variables

    int launcherPos = 0;
    boolean xdone = false;
    int all_the_way_down = -1050;
    int my_90_degrees = -850;
    int straight_up = -420;
    double servo_position = 1;
    boolean x_press___x_done_0 = false;
    boolean b_press_2 = false;
    boolean b_press = false;
    double motor_power_factor = 1;
    int offset = 0;
    boolean int_done = true;



    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        digital0 = hardwareMap.digitalChannel.get("digital0");
        motor0 = hardwareMap.dcMotor.get("motor0");
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor0B = hardwareMap.dcMotor.get("motor0B");
        motor2 = hardwareMap.dcMotor.get("motor2");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        motor3 = hardwareMap.dcMotor.get("motor3");
        park_assist = hardwareMap.dcMotor.get("motor3B");

        if (digital0.getState()) {
            int_done = true;
        } else {
            int_done = false;
        }
        motor_stuff();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                lancher_position();
                slow_mode();
                drive_robot();
                foundation_grabber(.5);
                launcher_drive();
                data_out();
                Pickingupblockled();
                lettingoutblockled();
                Forwardled();
                backwardled();
                Turnleftled();
                Turnrightled();
                Stationaryled();
            }
        }
    }

    /**
     * Describe this function...
     */

    private void foundation_grabber(double speed) {
        if (gamepad1.dpad_down) {
            servo_position = 0;
            motor_power_factor = 1;
        } else if (gamepad1.dpad_up) {
            servo_position = 1;
        } else if (gamepad1.dpad_right){

            park_assist.setPower(speed);

        } else if (gamepad1.dpad_left){

            park_assist.setPower(-speed);

        }else{

            park_assist.setPower(0);

        }

        servo0.setPosition(servo_position);
        servo1.setPosition(servo_position);

    }

    /**
     * Describe this function...
     */

    private void drive_robot() {

        if (gamepad1.back || gamepad2.back) {
            motor0.setPower(0);
            motor1.setPower(0);
        } else {
            motor0.setPower(((gamepad2.right_trigger - gamepad2.left_trigger) + gamepad2.left_stick_x) / motor_power_factor);
            motor1.setPower(((gamepad2.right_trigger - gamepad2.left_trigger) - gamepad2.left_stick_x) / motor_power_factor);
        }
    }

    /**
     * Describe this function...
     */
    private void lancher_position() {
        if (gamepad1.b) {
            b_press = true;
        }
        if (b_press && !int_done && !digital0.getState()) {
            motor0B.setPower(0.25);
        } else if (b_press && !int_done) {
            b_press = false;
            int_done = true;
            for (int count = 0; count < 500; count++) {
                if (gamepad1.back || gamepad2.back) {
                    break;
                }
            }
            motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad1.left_bumper) {
            offset += -5;
        } else if (gamepad1.right_bumper) {
            offset += 5;
        }
        if (gamepad1.back || gamepad2.back) {
            motor0B.setPower(0);
            launcherPos = motor0B.getCurrentPosition();
        } else if (int_done) {
            if (gamepad1.x) {
                x_press___x_done_0 = true;
            } else if (gamepad1.y) {
                launcherPos = my_90_degrees;
            } else if (gamepad1.b) {
                b_press_2 = true;
            }
            if (!digital0.getState() && motor0B.getCurrentPosition() < -20 && b_press_2) {
                launcherPos += 60;
            } else if (b_press_2) {
                b_press_2 = false;
                launcherPos = motor0B.getCurrentPosition();
            }
            if (motor0B.getCurrentPosition() >= all_the_way_down + 40 && x_press___x_done_0) {
                launcherPos += -70;
            } else if (x_press___x_done_0) {
                launcherPos = motor0B.getCurrentPosition();
                x_press___x_done_0 = false;
                xdone = true;
            }
            if (digital0.getState() && launcherPos >= -30 || launcherPos < all_the_way_down + 65 && motor0B.getCurrentPosition() < all_the_way_down + 65) {
                motor0B.setPower(0);
            } else {
                motor0B.setTargetPosition(launcherPos + offset);
                motor0B.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor0B.setPower(0.5);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void motor_stuff() {
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        motor0B.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0B.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor0B.setDirection(DcMotorSimple.Direction.REVERSE);
        park_assist.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo0.setDirection(Servo.Direction.REVERSE);
        servo0.setPosition(1);
        servo1.setPosition(1);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
    }

    /**
     * Describe this function...
     */
    private void data_out() {
        telemetry.addData("Left Power", motor0.getPower());
        telemetry.addData("Right Power", motor1.getPower());
        telemetry.addData("Rightservo", servo0.getPosition());
        telemetry.addData("Leftservo", servo1.getPosition());
        telemetry.addData("launcher pos.", motor0B.getCurrentPosition());
        telemetry.addData("launcher set pos.", launcherPos);
        telemetry.addData("block grabber position 1", motor2.getCurrentPosition());
        telemetry.addData("block grabber right power", motor3.getPower());
        telemetry.addData("block grabber left power", motor2.getPower());
        telemetry.addData("block grabber position 2", motor3.getCurrentPosition());
        telemetry.addData("controler", motor3.getCurrentPosition());
        telemetry.addData("controler", motor3.getCurrentPosition());
        telemetry.addData("slow mode", 123);
        telemetry.addData("offset", offset);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void slow_mode() {
        if (gamepad2.dpad_down) {
            motor_power_factor = 4;
        } else if (gamepad2.dpad_up) {
            motor_power_factor = 1;
        } else if (gamepad2.dpad_right || gamepad2.dpad_left) {
            motor_power_factor = 2;
        }
    }

    /**
     * Describe this function...
     */
    private void launcher_drive() {
        if (gamepad1.back || gamepad2.back) {
            motor2.setPower(0);
            motor3.setPower(0);
        } else {
            // The Y axis of a joystick ranges from -1 in its topmost position
            // to +1 in its bottommost position. We negate this value so that
            // the topmost position corresponds to maximum forward power.
            motor2.setPower(gamepad1.left_trigger / 1 - gamepad1.right_trigger / 3);
            motor3.setPower(gamepad1.left_trigger / 1 - gamepad1.right_trigger / 3);
        }
    }

    /**
     * Describe this function...
     */
    private void Stationaryled() {
        if (gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    /**
     * Describe this function...
     */
    private void Forwardled() {
        if (gamepad2.right_trigger > 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }

    /**
     * Describe this function...
     */
    private void Turnrightled() {
        if (gamepad2.left_stick_x < 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
        }
    }

    /**
     * Describe this function...
     */
    private void backwardled() {
        if (gamepad2.left_trigger > 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
    }

    /**
     * Describe this function...
     */
    private void Turnleftled() {
        if (gamepad2.left_stick_x > 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
        }
    }

    /**
     * Describe this function...
     */
    private void Pickingupblockled() {
        if (gamepad1.right_trigger > 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        }
    }

    /**
     * Describe this function...
     */
    private void lettingoutblockled() {
        if (gamepad1.left_trigger > 0) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
        }
    }
}