package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Johnny8 {
    private HardwareMap hwMap;

    LinearOpMode auton;

    OpMode teleop;

    public enum Drivetrain {
        MECHANUM,
        JOHNNY8,
        TEST
    }

    public enum Team {
        RED,
        BLUE
    }

    private Drivetrain drive;

    private Telemetry telem;

    //Definitions for global variables

    public DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    public DcMotorEx slideMotor;

    //[] means array
    public DcMotor[] allDriveMotors;

    //public CRServo //future necessary robot functions using servos
    private IMU imu;

    private IMU.Parameters parameters;

    public WebcamName webcamName;

    //Put any CONSTANTS here

    static final double Y_INCH_TICKS = 45;

    static final double X_INCH_TICKS = 45;

    static final double X_DEGREE_TICKS = 11.1; //may need to be changed

    static final double Y_DEGREE_TICKS = 11.1; //may need to be changed

    public Johnny8(OpMode opmode, Drivetrain drivetrain) {

        this.teleop = opmode;

        this.hwMap = opmode.hardwareMap;

        this.drive = drivetrain;

        this.telem = opmode.telemetry;

        //setUpHardware maps variables to their hardware object
        setupHardware();
    }

    public Johnny8(LinearOpMode opmode, Drivetrain type) {

        this.auton = opmode;

        hwMap = opmode.hardwareMap;

        telem = opmode.telemetry;

        drive = type;

        setupHardware();

        telem.addLine("init motor test");
    }

    public Johnny8(HardwareMap hardwareMap, Drivetrain drivetrain) {

        this.hwMap = hardwareMap;

        this.drive = drivetrain;

        setupHardware();
    }

    private void setupHardware() {

        //This switch statement is used to choose which drivetrain
        //depending on the drive variable
        switch (drive) {

            case JOHNNY8:

                motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
                motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
                motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
                motorBackRight = hwMap.dcMotor.get("motorBackRight");
                slideMotor = hwMap.get(DcMotorEx.class, "slideMotor");

                slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                slideMotor.setDirection(DcMotor.Direction.REVERSE);

                imu = hwMap.get(IMU.class, "imu");


                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));


                imu.initialize(parameters);
                //initialize touch sensor

                allDriveMotors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};


                break;

            case TEST:

                //setup motors for drivetrain
                motorFrontLeft = hwMap.dcMotor.get("motorFrontLeft");
                motorFrontRight = hwMap.dcMotor.get("motorFrontRight");
                motorBackLeft = hwMap.dcMotor.get("motorBackLeft");
                motorBackRight = hwMap.dcMotor.get("motorBackRight");

                //Reverse motors
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

                //Here would go any additional hardware devices for the robot

                imu = hwMap.get(IMU.class, "imu");

                parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

                imu.initialize(parameters);


                //camera setup!
                webcamName = hwMap.get(WebcamName.class, "eyeofjohnny6");
                allDriveMotors = new DcMotor[]{motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight};

                break;


        }
    }

    //Set motor power for all drivetrain motors on robot to 0

    //set powers for motors and positions for servos
    public void rest() {
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);

    }

    /*
    This function controls movement for the robot.
    @param x the x speed value
    @param y the y speed value
    @param turn the turn speed value
     */

    public void move(double x, double y, double turn) {

        switch (drive) {

            case JOHNNY8:
                //Denominator is the larget motor power (absolute value) or 1
                //This ensures all the powers maintain the same ratio, but only when
                //at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

                //Compute values for the power of each motor
                double frontLeftPower = (y + x + turn) / denominator;
                double backLeftPower = (y - x + turn) / denominator;
                double frontRightPower = (y - x - turn) / denominator;
                double backRightPower = (y + x - turn) / denominator;
                telem.addLine("frontLeft: " + frontLeftPower);
                telem.addLine("frontRight: " + frontRightPower);
                telem.addLine("backLeft: " + backLeftPower);
                telem.addLine("backRight: " + backRightPower);

                telem.addData("front left encoder:", motorFrontLeft.getCurrentPosition());
                telem.addData("front right encoder:", motorFrontRight.getCurrentPosition());
                telem.addData("back left encoder:", motorBackLeft.getCurrentPosition());
                telem.addData("back right encoder:", motorBackRight.getCurrentPosition());
                telem.addData("slide motor encoder:", slideMotor.getCurrentPosition());
                telem.addData("slide target", slideMotor.getTargetPosition());

                //Assign that motor power to each motor
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                break;

            case MECHANUM:

                //Denominator is the larget motor power (absolute value) or 1
                //This ensures all the powers maintain the same ratio, but only when
                //at least one is out of the range [-1, 1]
                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

                //Compute values for the power of each motor
                frontLeftPower = (y + x + turn) / denominator;
                backLeftPower = (y - x + turn) / denominator;
                frontRightPower = (y - x - turn) / denominator;
                backRightPower = (y + x - turn) / denominator;

                //Assign that motor power to each motor
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                break;


            case TEST:

                //Denominator is the larget motor power (absolute value) or 1
                //This ensures all the powers maintain the same ratio, but only when
                //at least one is out of the range [-1, 1]

                denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

                //Compute values for the power of each motor
                frontLeftPower = (y + x + turn) / denominator;
                backLeftPower = (y - x + turn) / denominator;
                frontRightPower = (y - x - turn) / denominator;
                backRightPower = (y + x - turn) / denominator;

                //Assign that motor power to each motor
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);

                break;

        }
    }


    public void resetYaw() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public void slideUp() {
        slideMotor.setPower(1);
    }


    public void slideTo(int tickTarget) {
        slideMotor.setTargetPosition(tickTarget);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.9);
    }

    public void slideLow() {
        slideTo(25);
    }

    public void slideMedium() {
        slideTo(1500);
    }

    public void slideHigh() {
        slideTo(4000);
    }

    public void slideHang() {
        slideTo(3000);
    }

    public void slideUpTick() {
        slideTo(slideMotor.getCurrentPosition() + 100);
    }

    public void slideDownTick() {
        slideTo(slideMotor.getCurrentPosition() - 100);
    }

    public void moveSlide(double speed) {
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(speed);
    }

    public void waitForMotors() {
        boolean finished = false;
        while (auton.opModeIsActive() && !finished && !auton.isStopRequested()) {
            if (motorFrontLeft.isBusy() || motorBackLeft.isBusy() || motorFrontRight.isBusy() || motorBackRight.isBusy()) {
                telem.addData("front left encoder:", motorFrontLeft.getCurrentPosition());
                telem.addData("front right encoder:", motorFrontRight.getCurrentPosition());
                telem.addData("back left encoder:", motorBackLeft.getCurrentPosition());
                telem.addData("back right encoder:", motorBackRight.getCurrentPosition());
                telem.addData("slide motor encoder:", slideMotor.getCurrentPosition());

                telem.update();
            } else {
                finished = true;
            }
        }
    }


    public void waitForSlideMotor() {
        boolean finished = false;
        while (!finished) {
            if (slideMotor.isBusy()) {
                telem.addData("slide motor encoder:", slideMotor.getCurrentPosition());
                telem.addData("slide target", slideMotor.getTargetPosition());
                telem.update();
            } else {
                finished = true;
            }
        }
    }


    public void resetSlideEncoder() {
        slideMotor.setPower(0);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}