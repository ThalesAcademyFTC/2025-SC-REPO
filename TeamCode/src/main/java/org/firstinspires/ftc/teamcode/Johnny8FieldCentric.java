package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class Johnny8FieldCentric extends LinearOpMode {
    public Johnny8 BigJ;
    @Override
    public void runOpMode() {
        BigJ = new Johnny8(this, Johnny8.Drivetrain.MECHANUM);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            BigJ.imu.initialize(parameters);
            waitForStart();

            while(opModeIsActive()){
                double y=-gamepad1.left_stick_y;
                double x=gamepad1.left_stick_x;
                double rx=gamepad1.right_stick_x;

                if (gamepad1.options){
                    BigJ.imu.resetYaw();
                }
                double botHeading = BigJ.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }


    }

}
