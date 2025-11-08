package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class if_practic extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        boolean abouton = gamepad1.a;
        double lefty = gamepad1.left_stick_y;
        double leftx = gamepad1.left_stick_x;

         if (abouton){
             telemetry.addData("a Bouton", "pressed");
         }
         else {
             telemetry.addData("a Bouton", "not pressed");
         }
         telemetry.addData("a Boutoun state",abouton);
         //first extemple //


        if(lefty < 0){
            telemetry.addData("left Stick", "is négative");
        }
        else if (lefty > 0.5) {
            telemetry.addData("left Stick", "is greater than 50 %");
        }
        else if (lefty > 0) {
            telemetry.addData("left Stick", "is greater than 0");
        }
        else {
            telemetry.addData("left stick ", "is Zero");
        }
        telemetry.addData("left Sick value",lefty);


        if(leftx < 0.1 && leftx >0.1){
            telemetry.addData("leftt stick ", "is in dead zone");
        }



    }
}
