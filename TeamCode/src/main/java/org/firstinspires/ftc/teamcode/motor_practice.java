package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class motor_practice extends OpMode {
    motor_sample benche = new motor_sample();

    @Override
    public void init() {
        benche.init(hardwareMap);


    }
    @Override
    public void loop() {
        benche.setMotorSpeed(0.5);
    }




}
