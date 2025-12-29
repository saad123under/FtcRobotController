package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class motor_sample extends OpMode {
    private DcMotor motor;
    public void init(HardwareMap hwMap){


        //motor code
        motor = hwMap.get(DcMotor.class,"DriverL");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorSpeed(double speed){

        motor.setPower(speed);

    }



}
