package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class gamepad_lesson extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        //run 50x par seconde//

        double speed_forward = -gamepad1.left_stick_y / 2.0;

            telemetry.addData("x",gamepad1.left_stick_x);
        telemetry.addData("y",gamepad1.left_stick_y);
        telemetry.addData("a",gamepad1.a);
    }
}
