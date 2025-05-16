package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Johnny9Teleop")
public class Johnny9Teleop extends OpMode {
    Johnny9 johnny9;

    //allows driver customization
    static final double STRAFE_FACTOR=1.1;

    @Override
    public void init(){
        johnny9=new Johnny9(this,Johnny9.Drivetrain.JOHNNY9);
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
