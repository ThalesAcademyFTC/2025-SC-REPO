package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.Johnny8.X_INCH_TICKS;
import static org.firstinspires.ftc.teamcode.Johnny8.Y_INCH_TICKS;



import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonFunctions {

    public Johnny8 johnny8;

    public void moveForwardInches(double inches,double speed){
        int tickTarget=(int) Math.round(inches*Y_INCH_TICKS);
        johnny8.resetDriveEncoders();
        for(DcMotor x:johnny8.allDriveMotors){
            x.setTargetPosition(tickTarget);
            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        johnny8.move(0,-speed,0);
        johnny8.waitForMotors();
        johnny8.resetDriveEncoders();
    }
    public void moveBackwardInches(double inches, double speed){
        moveForwardInches(-inches,-speed);
    }
    public void moveRightInches(double inches,double speed){
        //tickTarget = position of motors
        int tickTarget=(int) Math.round(-inches*Y_INCH_TICKS);
        johnny8.resetDriveEncoders();
        johnny8.motorFrontLeft.setTargetPosition(tickTarget);
        johnny8.motorFrontRight.setTargetPosition(-tickTarget);
        johnny8.motorBackLeft.setTargetPosition(-tickTarget);
        johnny8.motorBackRight.setTargetPosition(tickTarget);
        for (DcMotor x : johnny8.allDriveMotors) {

            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        johnny8.move(speed, 0, 0);
        johnny8.waitForMotors();

        johnny8.resetDriveEncoders();
    }

    public void moveLeftInches(double inches,double speed){
       //haha funny i used other function in function!!!! :)
        moveRightInches(-inches,-speed);
    }
    public void turnRightDegrees(double degrees, double speed){
        int tickTarget=(int) Math.round(degrees*X_INCH_TICKS);
        johnny8.resetDriveEncoders();
        johnny8.motorFrontLeft.setTargetPosition(tickTarget);
        johnny8.motorFrontRight.setTargetPosition(-tickTarget);
        johnny8.motorBackLeft.setTargetPosition(tickTarget);
        johnny8.motorBackRight.setTargetPosition(-tickTarget);
        for(DcMotor x:johnny8.allDriveMotors){
            x.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        johnny8.move(0,0,speed);
        johnny8.waitForMotors();
        johnny8.resetDriveEncoders();
    }
    public void turnLeftDegrees(double degrees, double speed){
        turnRightDegrees(-degrees,-speed);
    }
}
