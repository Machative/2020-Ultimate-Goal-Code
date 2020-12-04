package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class Teleop extends LinearOpMode {
    //Gamepad 2 Timers
    public static long atimer=0;

    float shooterpower=0.0f;

    public static double speed=25.0;
    public static boolean manualWobbleArm=false;
    public static int wobbleArmPos=0;
    public static int direction=0;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            //Drive Controls
            float ly = gamepad1.left_stick_y;
            float lx = gamepad1.left_stick_x;
            float ry = gamepad1.right_stick_y;
            float rx = gamepad1.right_stick_x;
            float gas = gamepad1.left_trigger;

            drive.setDrivePowers(lx,-ly,rx,-ry);

            //Mechanism Controls
            float wobble = gamepad2.right_stick_x; //Tighten/Loosen Wobble Grabber Servo
            float wobblearm = gamepad2.right_stick_y; //Manually adjust Wobble Grabber Arm
            boolean wobblearmpos = gamepad2.a;//Toggle Wobble Arm Position
            boolean liftUp = gamepad2.dpad_up; //Raise Ring Lift
            boolean liftDown = gamepad2.dpad_down; //Lower Ring Lift
            //float shooterangle = -gamepad2.left_stick_x; //Adjust Shooter Angle
            float shooter = gamepad2.left_stick_y;//Adjust Shooter Speed

            telemetry.addData("current pos: ",drive.wobbleGrabberArm.getCurrentPosition());
            telemetry.update();

            //Tighten/Loosen Wobble Grabber Servo
            drive.wobbleGrabber.setPower(wobble);

            /*//Adjust Shooter Angle
            drive.shooterStringLeft.setPower(shooterangle);
            drive.shooterStringRight.setPower(shooterangle);*/

            //Adjust Shooter Speed
            drive.setShooterSpeed(shooter);

            //Toggle Wobble Grabber Arm Position
            if(wobblearmpos && System.currentTimeMillis()-atimer>250){
                manualWobbleArm=false;
                if(wobbleArmPos==0) {wobbleArmPos=1;direction=0;}
                else if(wobbleArmPos==1 && direction==0) wobbleArmPos=2;
                else if(wobbleArmPos==1 && direction==1) wobbleArmPos=0;
                else if(wobbleArmPos==2) {wobbleArmPos=1;direction=1;}
                atimer=System.currentTimeMillis();
            }
            if(!manualWobbleArm) drive.setWobbleGrabberArmPosition(wobbleArmPos);

            //Manual Wobble Grabber Arm Control
            if(wobblearm < -0.1 || wobblearm > 0.1){
                manualWobbleArm=true;
                drive.wobbleGrabberArm.setTargetPosition(drive.wobbleGrabberArm.getCurrentPosition()+(int)(wobblearm*speed));
                drive.wobbleGrabberArm.setPower(1);
            }

            //Ring Elevator Control
            if(liftUp) {
                drive.ringElevatorDriver.setPower(0.5);
            }else if(liftDown){
                drive.ringElevatorDriver.setPower(-0.5);
            }else{
                drive.ringElevatorDriver.setPower(0);
            }
        }
    }
}
