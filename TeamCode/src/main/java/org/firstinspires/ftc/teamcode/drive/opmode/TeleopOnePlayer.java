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
public class TeleopOnePlayer extends LinearOpMode {
    //Gamepad 2 Timers
    public static long atimer=0;
    public static long btimer=0;
    public static long xtimer=0;
    public static long ytimer=0;
    public static long dpaddowntimer=0;
    public static long dpaduptimer=0;
    public static boolean shooterOn=false;

    float shooterpower=0.82f;

    public static boolean manualWobbleArm=false;
    public static int wobbleArmPos=-70;
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
            float gas = gamepad1.right_trigger+1.0f;
            boolean aimMode = gamepad1.right_bumper;

            float aim = (aimMode?0.3f:1);
            float aimStrafe = (aimMode?0.5f:1);
            drive.setDrivePowers(rx*0.5*gas*aimStrafe,-ly*0.5*gas*aim,lx*0.5*gas*aim,-ry*0.5*gas*aim);

            //Mechanism Controls
            //boolean wobbleGrab = gamepad2.y;//Wobble Grabber Servo
            //float wobblearm = gamepad2.right_stick_y; //Manually adjust Wobble Grabber Arm
            //boolean wobblearmpos = gamepad2.a;
            boolean intake = gamepad1.x;//Toggle Intake
            boolean shooter = gamepad1.b;//Toggle Shooter
            boolean shooterDec = gamepad1.dpad_down;
            boolean shooterInc = gamepad1.dpad_up;

            //Tighten/Loosen Wobble Grabber Servo
            /*if(wobbleGrab && System.currentTimeMillis()-ytimer>500){
                drive.wobbleGrabber.setPosition(1-drive.wobbleGrabber.getPosition());
                ytimer = System.currentTimeMillis();
            }*/

            //Toggle Intake
            if(intake && System.currentTimeMillis()-xtimer>250){
                drive.toggleIntake();
                xtimer = System.currentTimeMillis();
            }
            //Toggle Shooter
            if(shooter && System.currentTimeMillis()-btimer>250){
                shooterOn=!shooterOn;
                drive.ringPusher.setPosition(1-drive.ringPusher.getPosition());
                btimer = System.currentTimeMillis();
            }
            if(shooterOn){
                drive.setShooterSpeed(shooterpower);
            }else{
                drive.shooterOff();
            }
            telemetry.addData("shooter power", shooterpower);
            telemetry.update();
            if(shooterDec && System.currentTimeMillis()-dpaddowntimer>250 && shooterpower>0f){
                shooterpower-=0.01f;
                dpaddowntimer=System.currentTimeMillis();
            } else if(shooterInc && System.currentTimeMillis()-dpaduptimer>250 && shooterpower<1.0f){
                shooterpower+=0.01f;
                dpaduptimer=System.currentTimeMillis();
            }

            //Toggle Wobble Grabber Arm Position
            /*if(wobblearmpos && System.currentTimeMillis()-atimer>250){
                manualWobbleArm=false;
                if(wobbleArmPos==-70) wobbleArmPos=-700;
                else if(wobbleArmPos==-700) wobbleArmPos=-70;
                atimer=System.currentTimeMillis();
            }
            if(!manualWobbleArm) drive.setWobbleGrabberArmPosition(wobbleArmPos);

            //Manual Wobble Grabber Arm Control
            if(wobblearm < -0.1 || wobblearm > 0.1) {
                manualWobbleArm = true;
                drive.wobbleGrabberArm.setTargetPosition(drive.wobbleGrabberArm.getCurrentPosition() + (int) (wobblearm * speed));
                drive.wobbleGrabberArm.setPower(1);
            }*/
        }
    }
}