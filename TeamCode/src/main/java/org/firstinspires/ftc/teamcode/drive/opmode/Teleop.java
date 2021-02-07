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
    //Gamepad 1 Timers
    public static long atimer1=0;
    public static long joltTimer=0;
    public static boolean joltBool=false;
    //Gamepad 2 Timers
    public static long atimer2=0;
    public static long btimer2=0;
    public static long xtimer2=0;
    public static long ytimer2=0;
    public static long rbtimer2=0;
    public static long servoTimer=0;
    public static long dpaddowntimer=0;
    public static long dpaduptimer=0;
    public static boolean shooterOn=false;

    float shooterpower=0.82f;
    float wobbleArmSpeed=-50;

    public static boolean manualWobbleArm=false;
    public static int wobbleArmPos=-10;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            //Drive Controls
            float ly = gamepad1.left_stick_y;
            float lx = gamepad1.left_stick_x;
            float ry = gamepad1.right_stick_y;
            float rx = gamepad1.right_stick_x;
            float gas = gamepad1.right_trigger+1.0f;
            boolean jolt = gamepad1.a;
            boolean aimMode = gamepad1.right_bumper;

            float aim = (aimMode?0.3f:1);
            float aimStrafe = (aimMode?0.45f:1);
            drive.setDrivePowers(rx*0.5*gas*aimStrafe,-ly*0.5*gas*aim,lx*0.5*gas*aim,-ry*0.5*gas*aim);

            //Mechanism Controls
            float wobblearm = gamepad2.right_stick_y; //Manually adjust Wobble Grabber Arm Position
            boolean intake = gamepad2.x;//Toggle Intake
            boolean shooterMotors = gamepad2.y;//Toggle Shooter
            boolean shoot = gamepad2.right_bumper;//Shoot Ring w/ Shooter Servo
            boolean wobbleGrab = gamepad2.a;//Grab Wobble w/ Servo
            boolean wobbleToggle = gamepad2.b;//Toggle Wobble Grabber Arm Position
            boolean shooterDec = gamepad2.dpad_down;//Decrease Shooter Speed
            boolean shooterInc = gamepad2.dpad_up;//Increase Shooter Speed
            float intakeManual = gamepad2.left_stick_y;//Manually adjust Intake Speed/Direction

            //JOLT
            if(jolt && System.currentTimeMillis()-atimer1>250){
                drive.setDrivePowers(0,0,-1,0);
                joltTimer=System.currentTimeMillis();
                atimer1=System.currentTimeMillis();
            }
            if(System.currentTimeMillis()-joltTimer>50 && joltTimer>0){
                if(!joltBool){
                    drive.setDrivePowers(0,0,1,0);
                    joltTimer=System.currentTimeMillis();
                    joltBool=true;
                }else{
                    drive.setDrivePowers(0,0,0,0);
                    joltBool=false;
                    joltTimer=-1;
                }
            }

            //INTAKE
            if(intake && System.currentTimeMillis()-xtimer2>250 && Math.abs(intakeManual)<0.1){
                drive.toggleIntake();
                xtimer2 = System.currentTimeMillis();
            }else if(Math.abs(intakeManual)>0.1){
                drive.setIntakePower(-intakeManual);
            }

            //SHOOTER MOTORS
            if(shooterMotors && System.currentTimeMillis()-ytimer2>250){
                shooterOn=!shooterOn;
                ytimer2=System.currentTimeMillis();
            }
            if(shooterOn){
                drive.setShooterSpeed(shooterpower);
            }else{
                drive.shooterOff();
            }
            telemetry.addData("Shooter Power: ", Math.floor((shooterpower*100)+0.1f));
            telemetry.addData("Wobble Arm Pos: ",drive.wobbleGrabberArm.getCurrentPosition());

            telemetry.update();
            if(shooterDec && System.currentTimeMillis()-dpaddowntimer>250 && shooterpower>0f){
                shooterpower-=0.01f;
                dpaddowntimer=System.currentTimeMillis();
            } else if(shooterInc && System.currentTimeMillis()-dpaduptimer>250 && shooterpower<1.0f){
                shooterpower+=0.01f;
                dpaduptimer=System.currentTimeMillis();
            }

            //SHOOTER SERVO
            if(shoot && System.currentTimeMillis()-rbtimer2>250){
                drive.ringPusher.setPosition(1);
                servoTimer=System.currentTimeMillis();
                rbtimer2 = System.currentTimeMillis();
            }
            if(drive.ringPusher.getPosition()==1 && System.currentTimeMillis()-servoTimer>800) {
                drive.ringPusher.setPosition(0);
            }

            //WOBBLE GRABBER
            if(wobbleGrab && System.currentTimeMillis()-atimer2>500){
                drive.toggleWobbleGrabber();
                atimer2 = System.currentTimeMillis();
            }
            //WOBBLE GRABBER ARM
            if(wobbleToggle && System.currentTimeMillis()-btimer2>250){
                manualWobbleArm=false;
                if(wobbleArmPos==-10) wobbleArmPos=-350;
                else if(wobbleArmPos==-350) wobbleArmPos=-10;
                btimer2=System.currentTimeMillis();
            }
            if(!manualWobbleArm) drive.setWobbleArmPosition(wobbleArmPos);

            //MANUAL WOBBLE GRABBER ARM CONTROL
            if(wobblearm < -0.1 || wobblearm > 0.1) {
                manualWobbleArm = true;
                drive.setWobbleArmPosition(drive.wobbleGrabberArm.getCurrentPosition() + (int) (wobblearm * wobbleArmSpeed));
                drive.wobbleGrabberArm.setPower(1);
            }
        }
    }
}
