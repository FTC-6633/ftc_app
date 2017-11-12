package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;


/**
 * FTC6633 Technosaurus Rex Autonomous Mode Code.
 * Created by Alec Agayan and Roman Zrajevsky on 11/11/2017
 */

@Autonomous(name="ForwardAuto", group="Tutorial")
//@Disabled
public class ForwardAuto extends LinearOpMode {

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
    double clawOffset = 0;


    @Override
    public void runOpMode() {
        dc_drive_controller = hardwareMap.dcMotorController.get("drive_controller");
        dc_drive_controller2 = hardwareMap.dcMotorController.get("drive_controller2");

        dc_drive_left = hardwareMap.dcMotor.get("drive_left");
        dc_drive_right = hardwareMap.dcMotor.get("drive_right");
        dc_drive_left.setDirection(DcMotor.Direction.FORWARD);
        dc_drive_right.setDirection(DcMotor.Direction.REVERSE);

        dc_rear_left = hardwareMap.dcMotor.get("rear_left");
        dc_rear_right = hardwareMap.dcMotor.get("rear_right");
        dc_rear_left.setDirection(DcMotor.Direction.FORWARD);
        dc_rear_right.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        double power = -0.5;
        dc_drive_left.setPower(power);
        dc_drive_right.setPower(power);

        dc_rear_left.setPower(power);
        dc_rear_right.setPower(power);

        sleep(1400);

        power = 0;
        dc_drive_left.setPower(power);
        dc_drive_right.setPower(power);
        dc_rear_left.setPower(power);
        dc_rear_right.setPower(power);

    }

}
