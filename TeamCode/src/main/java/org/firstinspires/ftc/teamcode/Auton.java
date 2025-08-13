/*package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Johnny8.X_INCH_TICKS;
import static org.firstinspires.ftc.teamcode.Johnny8.Y_INCH_TICKS;



import com.qualcomm.robotcore.hardware.DcMotor;

public class Auton {
    public void moveForwardInches(double inches,double speed){
        int tickTarget=(int) Math.round(inches*Y_INCH_TICKS);
        Johnny8.resetDriveEncoders();
        for(DcMotor x:allDriveMotors){
            x.setTargetPosition(tickTarget);
            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        Johnny8.move(0,-speed,0);
        Johnny8.waitForMotors();
        Johnny8.resetDriveEncoders();
    }
    public void moveBackwardInches(double inches, double speed){
        moveForwardInches(-inches,-speed);
    }
    public void moveRightInches(double inches,double speed){
        int tickTarget=(int) Math.round(-inches*X_INCH_TICKS);
        Johnny8.resetDriveEncoders();
        Johnny8.motorFrontLeft.setTargetPosition(tickTarget);
        Johnny8.motorFrontRight.setTargetPosition(-tickTarget);
        Johnny8.motorBackLeft.setTargetPosition(-tickTarget);
        Johnny8.motorBackRight.setTargetPosition(tickTarget);
        for (DcMotor x : Johnny8.allDriveMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        Johnny8.move(speed, 0, 0);
        Johnny8.waitForMotors();

        Johnny8.resetDriveEncoders();
    }
    public void moveLeftInches(double inches,double speed){
        moveRightInches(-inches,-speed);
    }
}
*/