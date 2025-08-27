package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Johnny9Teleop")
public class Johnny8Teleop extends OpMode {
    Johnny8 johnny8;

    //allows driver customization
    static final double STRAFE_FACTOR=1.1;

    @Override
    public void init(){johnny8=new Johnny8(this,Johnny8.Drivetrain.JOHNNY8);johnny8.resetSlideEncoder();}
    @Override
    public void loop(){
        double y=gamepad1.left_stick_y;
        double x=gamepad1.left_stick_x;
        double rx=gamepad1.right_stick_x/2;
        telemetry.addData("x:",x);
        telemetry.addData("y:",y);
        telemetry.addData("turn(rx):",rx);
        telemetry.update();

        y*=y;
        if (gamepad1.left_stick_y > 0){
            y=-y;
        }
        x*=x;
        if (gamepad1.left_stick_x < 0){
            x=-x;

        }
        johnny8.move(x,y,rx);

       /* if(gamepad2.dpad_up) {
            johnny8.slideHigh();
        }else if(gamepad2.dpad_down){
            johnny8.slideLow();
        }else if(gamepad2.dpad_right){
            johnny8.slideMedium();
        }else if(gamepad2.dpad_left){
            johnny8.slideHang();
        }*/

    }

}
