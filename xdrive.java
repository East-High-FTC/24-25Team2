package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "xdrive")
public class xdrive extends LinearOpMode {
        /* 
            tr = 0
            tl = 1
            bl = 2
            br = 3
        */
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor br = null;
    private DcMotor bl = null;
    private DcMotor climberspool = null;
    private DcMotor intake = null;
    private Servo drone_servo = null;
    private IMU imu = null;
    
    private double padx = 0.0;
    private double pady = 0.0;
    private double rot = 0.0;
    
    private double current_rot = 0.0;
    private double topleft_power = 0.0;
    private double topright_power = 0.0;
    private double bottomleft_power = 0.0;
    private double bottomright_power = 0.0;
    
    private int drive_mode = 1; // 0 = tank drive 1 = absolute drive
    //private double motor_power = 1.0; // fraction of power for motors
    private double motor_power = 0.5; // fraction of power for motors
    private double motor_low = 0.4;
    
    private boolean intake_open = false; // fraction of power for motors
    
    /**
    * * * * * This function is executed when this OpMode is selected from the Driver Station.
    * * * * */
    @Override
    public void runOpMode() {
        tl = hardwareMap.get(DcMotor.class, "front_left");
        tr = hardwareMap.get(DcMotor.class, "front_right");
        br = hardwareMap.get(DcMotor.class, "back_right");
        bl = hardwareMap.get(DcMotor.class, "back_left");
        climberspool = hardwareMap.get(DcMotor.class, "climberspool");
        intake = hardwareMap.get(DcMotor.class, "intake");
        drone_servo = hardwareMap.get(Servo.class, "drone");
        
        initMotor();
        initIMU();
        
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                current_rot = getHeading();
                
                handleController();
                calculatePower();
                
                
                updateMotors();
                sendTelemetry();
            }
        }
    }
    
    private void initIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        imu.resetYaw();
    }
    
    private void initMotor() {
        
        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climberspool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    private void handleController() {
        //padx = pady = rot = 0;
        
        // driver 1 only 
        // change robot speed
        if (gamepad1.right_bumper == true) {
            motor_power = motor_low;
        } else {
            //motor_power = 1.0;
            motor_power = 0.7;
        }
        
        // change as needed
        rot = -gamepad1.right_stick_x;
        padx = -gamepad1.left_stick_x;
        pady = -gamepad1.left_stick_y;
        rot *= -1;
        
        double deadzone = 0.03;
        rot = Math.abs(gamepad1.right_stick_x) >= deadzone ? -gamepad1.right_stick_x : 0;
        padx = Math.abs(gamepad1.left_stick_x) >= deadzone ? -gamepad1.left_stick_x : 0;
        pady = Math.abs(gamepad1.left_stick_y) >= deadzone ? -gamepad1.left_stick_y : 0;
        rot *= -1;
        
        
        if (gamepad1.x) {
            drive_mode = 1;
            imu.resetYaw();
        }
        
        if (gamepad1.a) {
            drive_mode = 0;
        }
        
        // both drivers
        if (gamepad1.b || gamepad2.b) {
            drone_servo.setPosition(1);
        } else {
            drone_servo.setPosition(0);
        }
        
        if (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) {
          // Run intake
          intake.setPower(2);
        } else {
          intake.setPower(0);
        }
        
        if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
          // Run outtake
          intake.setPower(-1);
        } else {
          intake.setPower(0);
        }
        
        if (gamepad1.dpad_down || gamepad2.dpad_down) {
          // lower arm
          climberspool.setPower(1);
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
          // raise arm
          climberspool.setPower(-1);
        } else {
                climberspool.setPower(0);
        }
    }
    
    // private void controlDrone(double val) {
    //     drone_servo.setPosition(val);
    // }
    
    private void updateMotors() {
        tl.setPower(topleft_power);
        tr.setPower(topright_power);
        br.setPower(bottomright_power);
        bl.setPower(bottomleft_power);
    }
    
    private void calculatePower() {
        if (drive_mode == 0) {
            current_rot = 0;
        }
        
        double new_padx = Math.cos(current_rot) * padx - Math.sin(current_rot) * pady;
        double new_pady = Math.sin(current_rot) * padx + Math.cos(current_rot) * pady;
        
        topleft_power = -new_pady + new_padx - rot;
        topright_power = new_pady + new_padx - rot;
        bottomleft_power = -new_pady - new_padx - rot;
        bottomright_power = new_pady - new_padx - rot;
        
        topleft_power *= motor_power;
        topright_power *= motor_power;
        bottomleft_power *= motor_power;
        bottomright_power *= motor_power;
    }
    
    
    private void sendTelemetry() {
        telemetry.addData("drive mode: ", (drive_mode == 0) ? "tank" : "abs");
        telemetry.addData("curr_rot", "%1f", current_rot);
        telemetry.addData("motor power: ", motor_power);
        //telemetry.addData("controlhub voltage", ControlHub_VoltageSensor.getVoltage());
        //telemetry.addData("expandhub voltage", ExpansionHub2_VoltageSensor.getVoltage());
        //telemetry.addData("bl", ((DcMotorEx) bl).getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("br", ((DcMotorEx) br).getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("fl", ((DcMotorEx) fl).getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("fr", ((DcMotorEx) fr).getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("intake", ((DcMotorEx) intake).getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("climber", ((DcMotorEx) climberspool).getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
    
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }
}
