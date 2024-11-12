package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.XdriveAuto.COLOR;

@Autonomous(name="blue far park", group="Robot")
public class BlueFarPark extends LinearOpMode {

    XdriveAuto robot = new XdriveAuto(this, COLOR.BLUE);
    public double turn_power = 0.2;
    public double drive_power = 0.2;
    
    @Override
    public void runOpMode() {
        robot.autoInit(hardwareMap);
        
        waitForStart();
        
        double tmp = 0.0;
        //turn_power *= robot.idle(drive_power, turn_power);
        tmp = robot.idle(drive_power, turn_power);
        if (tmp == 0) {
            robot.driveLine(Math.PI, 20, drive_power * 2, robot.getHeading());
            robot.driveLine(-Math.PI / 2, 20, drive_power * 2, robot.getHeading());
            robot.driveLine(Math.PI, 22, drive_power * 2, robot.getHeading());
            robot.driveLine(Math.PI / 2, 100, drive_power * 2, robot.getHeading());
        } else {
            robot.driveLine(Math.PI, 43, drive_power, robot.getHeading());
            robot.driveLine(Math.PI / 2, 80, drive_power * 2, robot.getHeading());
            robot.driveLine(0, 3, drive_power * 2, robot.getHeading());
        }
        
        if (tmp == 0) {
            turn_power *= robot.calc_rot_dir_forback(-Math.PI);
        } else {
            turn_power *= tmp;
        }
        robot.rotate(Math.PI, turn_power/ 2, Math.PI / 540, 1);
        robot.outtake(0.7);
    }
}