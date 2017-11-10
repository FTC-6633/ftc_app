package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;


/**
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
        leftClawPosition = leftClaw.getPosition();
        leftClaw.setPosition(leftClawPosition);

        rightClaw = hardwareMap.get(Servo.class, "claw2");
        rightClawPosition = rightClaw.getPosition();
        rightClaw.setPosition(rightClawPosition);

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

    }

    /**
     * This method continuously gets called by the software. This is where
     * main event loop is and where we will control everything.
     */
    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y;
        double turn  = -gamepad1.right_stick_x;

        telemetry.addData("Drive", drive);
        telemetry.addData("Turn", drive);
        telemetry.addData("LSX", gamepad1.left_stick_x);
        telemetry.addData("RSY", gamepad1.right_stick_y);
        telemetry.update();


        double leftPower    = Range.clip((drive + turn/2)/2, -1.0, 1.0) ;
        double rightPower   = Range.clip((drive - turn/2)/2, -1.0, 1.0) ;

        dc_drive_left.setPower(leftPower);
        dc_drive_right.setPower(rightPower);

        dc_rear_left.setPower(leftPower);
        dc_rear_right.setPower(rightPower);

        // Lifting things
        double clawOffset = Range.clip(gamepad1.right_trigger, -0.5, 0.5);
        leftClaw.setPosition(clawOffset);

        double liftPower = Range.clip(gamepad1.right_stick_y, -1.0, 1.0);
        liftMotor.setPower(liftPower);
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
