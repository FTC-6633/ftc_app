package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;


/**
 * FTC6633 Technosaurus Rex Driver-Controlled Mode Code.
 * Created by FTC6633 on 10/13/2017.
 */

@TeleOp(name="Gamepad", group="Tutorial")
//@Disabled
public class Gamepad extends OpMode {

    /*****************************************
     *  Motor Controllers
     */
    private DcMotorController dc_drive_controller;
    private DcMotorController dc_drive_controller2;
    private DcMotorController lift_controller;
    private ServoController claw_controller;

    /*****************************************
     *  Motors Userd
     */
    private DcMotor dc_drive_left;
    private DcMotor dc_drive_right;
    private DcMotor dc_rear_left;
    private DcMotor dc_rear_right;

    private DcMotor liftMotor;

    private Servo leftClaw;
    private Servo rightClaw;

    double leftClawPosition = 0;
    double rightClawPosition = 0;

    final double CLAW_SPEED = 0.02;
    final double CLAW_LIMIT = 0.60;

    double clawOffset = 0;


    @Override
    public void init() {
        dc_drive_controller = hardwareMap.dcMotorController.get("drive_controller");
        dc_drive_controller2 = hardwareMap.dcMotorController.get("drive_controller2");
        //lift_controller = hardwareMap.get(MatrixDcMotorController.class, "lift_controller");
        lift_controller = hardwareMap.dcMotorController.get("lift_controller");

        // Define claw controller
        claw_controller = hardwareMap.get(ServoController.class, "claw_controller");
        // Enable Servos
        claw_controller.pwmEnable();

        dc_drive_left = hardwareMap.dcMotor.get("drive_left");
        dc_drive_right = hardwareMap.dcMotor.get("drive_right");
        dc_drive_left.setDirection(DcMotor.Direction.FORWARD);
        dc_drive_right.setDirection(DcMotor.Direction.REVERSE);

        dc_rear_left = hardwareMap.dcMotor.get("rear_left");
        dc_rear_right = hardwareMap.dcMotor.get("rear_right");
        dc_rear_left.setDirection(DcMotor.Direction.FORWARD);
        dc_rear_right.setDirection(DcMotor.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "claw1");
        leftClaw.setDirection(Servo.Direction.REVERSE);
        leftClaw.setPosition(0);
        leftClawPosition = leftClaw.getPosition();

        rightClaw = hardwareMap.get(Servo.class, "claw2");
        rightClaw.setPosition(0);
        rightClawPosition = rightClaw.getPosition();

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
    }

    /**
     * This method continuously gets called by the software. This is where
     * main event loop is and where we will control everything.
     */
    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn  = -gamepad1.left_stick_x;

        double leftPower    = Range.clip((drive + turn/2)/2, -1.0, 1.0) ;
        double rightPower   = Range.clip((drive - turn/2)/2, -1.0, 1.0) ;

        dc_drive_left.setPower(leftPower);
        dc_drive_right.setPower(rightPower);

        dc_rear_left.setPower(leftPower);
        dc_rear_right.setPower(rightPower);

        // Grabbing things
        if (gamepad1.right_bumper) {
            clawOffset += CLAW_SPEED;
        } else if (gamepad1.left_bumper) {
            clawOffset -= CLAW_SPEED;
        }

        // Claw differential is to help the two arms close without clashing with each other
        // Because of the differential, they are offset by a small amount.
        double CLAW_DIFF = 0.01;
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            clawOffset = Range.clip(clawOffset, CLAW_DIFF, CLAW_LIMIT);
            leftClaw.setPosition(clawOffset+CLAW_DIFF);
            rightClaw.setPosition(clawOffset-CLAW_DIFF);
        }

        // The code below allows the lift to be operated via D-Pad up and down buttons
        if (gamepad1.dpad_down)
        {
            liftMotor.setPower(0.75);
        }
        else if (gamepad1.dpad_up)
        {
            liftMotor.setPower(-0.75);
        }
        else {
            double liftPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
            liftMotor.setPower(liftPower);
        }

        // This information is helpful for debugging
        leftClawPosition = leftClaw.getPosition();
        rightClawPosition = rightClaw.getPosition();

        telemetry.addData("clawOffset", clawOffset);
        telemetry.addData("leftClaw", leftClawPosition);
        telemetry.addData("rightClaw", rightClawPosition);
        telemetry.update();



    }

    /**
     * This is the stop function.
     */
    @Override
    public void stop() {
        dc_drive_left.setPower(0);
        dc_drive_right.setPower(0);
    }

}
