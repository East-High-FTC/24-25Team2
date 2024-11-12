package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.XdriveAuto.COLOR;

@Autonomous(name="red close park", group="Robot")

public class RedClosePark extends LinearOpMode {
    XdriveAuto robot = new XdriveAuto(this, COLOR.RED);
    public double turn_power = 0.2;
    public double drive_power = 0.2;
    
    @Override
    public void runOpMode() {
        robot.autoInit(hardwareMap);
        
        waitForStart();
        
        double tmp = 0.0;
        //turn_power *= robot.idle(drive_power, turn_power);
        tmp = robot.idle(drive_power, turn_power);
        robot.driveLine(-Math.PI / 2, 37, drive_power, robot.getHeading());
        if (tmp == 0) {
            turn_power *= robot.calc_rot_dir_forback(-Math.PI);
        } else {
            turn_power *= tmp;
        }
        robot.rotate(Math.PI, turn_power / 2, Math.PI / 500, 1);
        robot.outtake(0.7);
    }
}
