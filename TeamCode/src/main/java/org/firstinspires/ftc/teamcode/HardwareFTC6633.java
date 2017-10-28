package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Alec, Aarush, Justin, and Roman on 10/2/2017.
 */

public class HardwareFTC6633 {

    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    HardwareMap map = null;

    public void init(HardwareMap aMap)
    {
        map = aMap;
    }


}
