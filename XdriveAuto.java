package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Disabled

public class XdriveAuto extends XdriveRobot {
    enum POS {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    
    enum COLOR {
        BLUE,
        RED,
        NONE,
    }
    
    private LinearOpMode opmode = null;
    
    
    private ElapsedTime time = null;
    private ColorSensor cs = null;
    
    private int tl_target = 0;
    private int tr_target = 0;
    private int bl_target = 0;
    private int br_target = 0;
    
    private POS prop_pos = POS.NONE;
    private COLOR alliance = COLOR.NONE;
    
    static final double     COUNTS_PER_MOTOR_REV    = 28;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // No External Gearing = 1
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     COUNTS_PER_REV          = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     RPM                     = 6000 / DRIVE_GEAR_REDUCTION;
    
    public XdriveAuto(LinearOpMode opmode, COLOR color) {
        this.opmode = opmode;
        this.alliance = color;
    }
    
    public void autoInit(HardwareMap map) {
        cs = map.get(ColorSensor.class, "cs");
        time = new ElapsedTime();
        init(map, true);
    }
    
    // calculate rotation used for forward / backward alignment
    public double calc_rot_dir_forback(double angle) {
        if (getHeading() < angle) {
            return 1;
        } else if (getHeading() > angle) {
            return -1;
        }
        return 0;
    }
    
    public double idle(double drive_power, double turn_power) {
        double turn_val = 60.0 * Math.PI / 180;
        //double turn_val = 55.0 * Math.PI / 180;
        //double turn_val = 62.5 * Math.PI / 180;
        
        intake.setPower(0.7);
        opmode.sleep(100);
        intake.setPower(0);
        
        driveLine(0, 26, drive_power, getHeading() + Math.PI);
        
        do {
            if (foundProp(1)) {
                opmode.telemetry.addData("found it", true);
                opmode.telemetry.update();
                prop_pos = POS.CENTER;
                break;
            }
            
            rotate(-turn_val, turn_power, 0, 1);
            if (foundProp(1)) {
                opmode.telemetry.addData("found it", true);
                opmode.telemetry.update();
                prop_pos = POS.RIGHT;
                break;
            }
            
            rotate(turn_val, turn_power, 0, 1);
            if (foundProp(1)) {
                opmode.telemetry.addData("found it", true);
                opmode.telemetry.update();
                prop_pos = POS.LEFT;
                break;
            }
            // change later if needed
            prop_pos = POS.LEFT;
            
            break;
        } while (prop_pos != POS.NONE); 
        
        if (prop_pos != POS.NONE) {
            switch (prop_pos) {
                case RIGHT: 
                    rotate(Math.PI / 2 + Math.PI / 10, -turn_power, Math.PI / 360, 1);
                    driveLine(-Math.PI / 2, 3, drive_power, getHeading());
                    outtake(0.7);
                    driveLine(Math.PI / 2, 3, drive_power, getHeading());
                    turn_power = 1;
                    break;
                case LEFT:
                    rotate(-Math.PI / 2, turn_power, Math.PI / 360, 1);
                    driveLine(Math.PI / 2, 2, drive_power, getHeading());
                    outtake(0.7);
                    driveLine(-Math.PI / 2, 2, drive_power, getHeading());
                    // faster turn towards front
                    turn_power = -1;
                    break;
                case CENTER:
                    rotate(Math.PI, turn_power, 0, 1);
                    driveLine(-Math.PI / 2, 5, drive_power, getHeading());
                    // may need to drive foreward a little more
                    driveLine(Math.PI, 2, drive_power, getHeading());
                    outtake(0.7);
                    driveLine(0, 2, drive_power, getHeading());
                    driveLine(Math.PI / 2, 5, drive_power, getHeading());
                    turn_power = 0;
                    break;
            }
        }
        
        driveLine(Math.PI, 20, drive_power, getHeading() + Math.PI);
        return turn_power;
    }
    
    public void outtake(double power) {
        intake.setPower(-power);
        opmode.sleep(1000);
        intake.setPower(0);
    }
    
    public boolean foundProp(double secs) {
        if (!opmode.opModeIsActive()) {
            return false;
        }
        
        boolean objectFound = false;
        double weight = 2.0/3.0;
        
        time.reset();
        while (time.seconds() <= secs && !objectFound) {
            if (alliance == COLOR.RED) {
                objectFound = cs.red() >= (cs.green() + cs.blue()) * weight && cs.red() > 10;
            }
            if (alliance == COLOR.BLUE) {
                // will need changing later
                objectFound = cs.blue() >= (cs.green() + cs.red()) * weight && cs.blue() > 10;
            }
            
            opmode.telemetry.addData("time", time.seconds());
            opmode.telemetry.addData("objectFound", objectFound);
            opmode.telemetry.addData("RGB: ", "%7d, %7d, %7d", cs.red(), cs.green(), cs.blue());
            if (alliance == COLOR.RED) {
                opmode.telemetry.addData("algo", cs.green() + cs.blue());
                opmode.telemetry.addData("weighted", cs.green() + cs.blue() * 2/3);
            }
            
            if (alliance == COLOR.BLUE) {
                opmode.telemetry.addData("algo", cs.green() + cs.red());
                opmode.telemetry.addData("weighted", cs.green() + cs.red() * 2/3);
            }
            opmode.telemetry.update();
        }
        return objectFound;
    }
    
    // if error is 0, error defaults to Math.PI / 180
    // if imu is used, rotates with 0 being directly in front, positive values are to the left of robot
    public void rotate(double rot, double speed, double error, int use_imu) {
        double curr_rot = 0;
        int mult = -1;
        if (error == 0) {
            error = Math.PI / 180;
        }
        
        if (rot < 0) {
            mult = 1;
        }
        
        double base = getHeading();
        curr_rot = getHeading();
        setPower(0, 0, 0, speed * mult, 1.0);
        updateMotors(tl_power, tr_power, bl_power, br_power);
        
        double max = (rot + error);
        double min = (rot - error);
        
        while (!(-mult * curr_rot >= min && -mult * curr_rot <= max)) {
            curr_rot = -mult * getHeading();
            if (use_imu == 0) {
                curr_rot -= -mult * base;
            }
            
            opmode.telemetry.addData("mode", use_imu == 1 ? "abs" : "rel");
            opmode.telemetry.addData("cur", radToDeg(curr_rot * -mult));
            opmode.telemetry.addData("rotating to", "%7f", rot * 180 / Math.PI);
            opmode.telemetry.addData("max", (rot + error) * 180 / Math.PI);
            opmode.telemetry.addData("min", (rot - error) * 180 / Math.PI);
            opmode.telemetry.addData("error", radToDeg(error));
            opmode.telemetry.addData("pow", tl_power);
            sendTelemetry();
        }
        updateMotors(0, 0, 0, 0);
        
    }
    
    public void driveLine(double angle, double dist, double speed, double heading) {
        if (opmode.opModeIsActive()) {
            // change if needed
            //double padx = -Math.cos(angle + Math.PI / 2);
            double padx = Math.cos(angle + Math.PI / 2);
            double pady = Math.sin(angle + Math.PI / 2);
            
            double rot = 0.0;
            
            int moveCounts = (int)(dist * COUNTS_PER_INCH * Math.sqrt(2) / 2);
            
            rot = heading;
            
            setPower(padx, pady, rot, 0.0, 1.0);
            
            tl_target    = (int)(tl.getCurrentPosition() + (tl_power * moveCounts));
            tr_target    = (int)(tr.getCurrentPosition() + (tr_power * moveCounts));
            bl_target    = (int)(bl.getCurrentPosition() + (bl_power * moveCounts));
            br_target    = (int)(br.getCurrentPosition() + (br_power * moveCounts));
            
            
            tl.setTargetPosition(tl_target);
            tr.setTargetPosition(tr_target);
            bl.setTargetPosition(bl_target);
            br.setTargetPosition(br_target);
            
            tl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            setPower(padx, pady, rot, 0.0, speed);
            updateMotors(tl_power, tr_power, br_power, bl_power);
            
            while (opmode.opModeIsActive() 
            && (tl.isBusy() 
            && tr.isBusy() 
            && br.isBusy() 
            && bl.isBusy())) {
                
                opmode.telemetry.addData("angle", radToDeg(angle));
                opmode.telemetry.addData("tl", ": %7d %7d at %7f", tl.getCurrentPosition(), tl.getTargetPosition(), tl_power);
                opmode.telemetry.addData("tr", ": %7d %7d at %7f", tr.getCurrentPosition(), tr.getTargetPosition(), tr_power);
                opmode.telemetry.addData("bl", ": %7d %7d at %7f", bl.getCurrentPosition(), bl.getTargetPosition(), bl_power);
                opmode.telemetry.addData("br", ": %7d %7d at %7f", br.getCurrentPosition(), br.getTargetPosition(), br_power);
                opmode.telemetry.update();
            }
            
            updateMotors(0.0, 0.0, 0.0, 0.0);
            
            tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    
    private void sendTelemetry() {
        opmode.telemetry.addData("imu", "%7f", radToDeg(getHeading()));
        opmode.telemetry.update();
    }
}
