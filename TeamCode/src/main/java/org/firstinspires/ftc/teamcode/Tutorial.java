package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;


/**
 * Created by Alec on 10/13/2017.
 */

@TeleOp(name="Tutorial", group="Tutorial")
//@Disabled
public class Tutorial extends OpMode {

    /*****************************************
     *  Motor Controllers
     */
    private DcMotorController dc_drive_controller;

    /*****************************************
     *  Motors Userd
     */
    private DcMotor dc_drive_left;
    private DcMotor dc_drive_right;


    @Override
    public void init() {
        dc_drive_controller = hardwareMap.dcMotorController.get("drive_controller");
        dc_drive_left = hardwareMap.dcMotor.get("drive_left");
        dc_drive_right = hardwareMap.dcMotor.get("drive_right");
    }

    @Override
    public void loop() {
        dc_drive_left.setPower(0.5);
        dc_drive_right.setPower(0.5);
    }

    @Override
    public void stop() {
        dc_drive_left.setPower(0);
        dc_drive_right.setPower(0);
    }

}
