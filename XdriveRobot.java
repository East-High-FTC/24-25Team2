package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
// imu 
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled

public class XdriveRobot {
    /* 
        control hub
        tr = 0
        tl = 1
        bl = 2
        br = 3
        rev internal IMU BNO055 = 0
        
        expansion hub
        climberspool = 0 NeveRest 40 Gearmotor
        intake = 3 NeveRest 45 Gearmotor
        drone = 5 Servo
        i2c bus 0 
        cs = 0 Rev Color/Range Sensor
    */ 
    public DcMotor         tl   = null;
    public DcMotor         tr  = null;
    public DcMotor         bl   = null;
    public DcMotor         br  = null;
    //public DcMotor intake = null;
    public IMU imu = null;
    
    
    public double tl_power = 0;
    public double tr_power = 0;
    public double bl_power = 0;
    public double br_power = 0;
    
    HardwareMap map = null;
    
    // for mapping func
    final static double EPSILON = 1e-12;
    
    public XdriveRobot() {
        
    }
    
    public void init(HardwareMap hwmap, boolean using_encoders) {
        map = hwmap;
        
        tl  = map.get(DcMotor.class, "front_left");
        tr = map.get(DcMotor.class, "front_right");
        bl  = map.get(DcMotor.class, "back_left");
        br = map.get(DcMotor.class, "back_right");
        //intake = map.get(DcMotor.class, "intake");
        
        if (using_encoders) {
            tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
        
        tl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // end motor config
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        imu = map.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        imu.resetYaw();
    }
    
    // if using encoders, use rpm, not fraction of power
    public void setPower(double padx, double pady, double current_rot, double rot, double power) {
        double new_padx = Math.cos(current_rot) * padx - Math.sin(current_rot) * pady;
        double new_pady = Math.sin(current_rot) * padx + Math.cos(current_rot) * pady;
        
        tl_power = -new_pady + new_padx - rot;
        tr_power = new_pady + new_padx - rot;
        bl_power = -new_pady - new_padx - rot;
        br_power = new_pady - new_padx - rot;
        
        tl_power *= power;
        tr_power *= power;
        bl_power *= power;
        br_power *= power;
    }
    
    public void updateMotors(double topleft_power, double topright_power, 
            double bottomright_power, double bottomleft_power) {
        // uncomment if mapping values needed
        topleft_power = map(topleft_power, 0, Math.sqrt(2), 0, 1.0);
        topright_power = map(topright_power, 0, Math.sqrt(2), 0, 1.0);
        bottomright_power = map(bottomright_power, 0, Math.sqrt(2), 0, 1.0);
        bottomleft_power = map(bottomleft_power, 0, Math.sqrt(2), 0, 1.0);
        
        tl.setPower(topleft_power);
        tr.setPower(topright_power);
        br.setPower(bottomright_power);
        bl.setPower(bottomleft_power);
    }
    
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }
    
    public double radToDeg(double rad) {
        return (180 / Math.PI) * rad;
    }
    
    public static double map(double valueCoord1, double startCoord1, double endCoord1, double startCoord2, double endCoord2) {
        if (Math.abs(endCoord1 - startCoord1) < EPSILON) {
            throw new ArithmeticException("/ 0");
        }
    
        double offset = startCoord2;
        double ratio = (endCoord2 - startCoord2) / (endCoord1 - startCoord1);
        return ratio * (valueCoord1 - startCoord1) + offset;
    }
}
