package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Johnny9Teleop")
public class Johnny8Teleop extends OpMode {
    Johnny8 johnny8;

    //allows driver customization
    static final double STRAFE_FACTOR=1.1;

    @Override
    public void init(){
        johnny8=new Johnny8(this,Johnny8.Drivetrain.JOHNNY8);
    }

    @Override
    public void loop(){
        double y=gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x;

        telemetry.update();

        y*=y;
        if (gamepad1.left_stick_y > 0){
            y=-y;
        }
        x*=x;
        if (gamepad1.left_stick_x < 0){
            x=-x;
        }

    }

}
