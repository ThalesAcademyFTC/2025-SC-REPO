package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU ;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Johnny8FieldCentric extends LinearOpMode {
    public Johnny8 BigJ;
    @Override
    public void runOpMode() {
        BigJ = new Johnny8(this, Johnny8.Drivetrain.MECHANUM);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);
            waitForStart();

            while(opModeIsActive()){
                double y=-gamepad1.left_stick_y;
                double x=gamepad1.left_stick_x;
                double rx=gamepad1.right_stick_x;

                if (gamepad1.options){
                    BigJ.imu.resetYaw();
                }
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double rotateX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotateY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                double denominator = Math.max(Math.abs(rotateY) + Math.abs(rotateX) + Math.abs(rx), 1);
                double frontLeftPower = (int) ((rotateY + rotateX + rx) / denominator);
                double backLeftPower =(int) (rotateY - rotateX + rx) / denominator;
                double frontRightPower =(int) (rotateY - rotateX - rx) / denominator;
                double backRightPower =(int) (rotateY + rotateX - rx) / denominator;

                BigJ.motorFrontLeft.setPower(frontLeftPower);
                BigJ.motorBackLeft.setPower(backLeftPower);
                BigJ.motorFrontRight.setPower(frontRightPower);
                BigJ.motorBackRight.setPower(backRightPower);

            }



    }

}
