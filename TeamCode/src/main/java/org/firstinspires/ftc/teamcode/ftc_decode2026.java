package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ftc_decode2026 extends OpMode {
    private DcMotor frontright , backright ;
    double throttle;
    double spin;
    @Override
    public void init() {
        frontright = hardwareMap.get(DcMotor.class,"driverL");
        backright = hardwareMap.get(DcMotor.class, "driverR");

        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.FORWARD);

    }


    @Override
    public void loop() {
        double leftPower = throttle + spin ;
        double rightPower = throttle - spin;
        double largest = Math.max(Math.abs(leftPower) ,Math.abs(rightPower));
        if(largest > 1.0){
            leftPower /= largest;
            rightPower /= largest;

        }
        frontright.setPower(leftPower);
        backright.setPower(rightPower);
    }


}
