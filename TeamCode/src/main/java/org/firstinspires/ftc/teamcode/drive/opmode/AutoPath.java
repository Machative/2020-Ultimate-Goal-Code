package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.HashMap;
import java.util.List;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class AutoPath extends OpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AVVIoiT/////AAABmX81rTPW60hcgKTP12YL9sFIHAXL7WyR1JI578v+YFJG/JSwjny6iEWiHEZ+twbt7HQ61pyg3A4/CCjpG1/u6VC6N2uK5bnWgFzeIHRESoUVX0pbphXVmkJ8NQmi9ZdKeNKV2ZgnM++ZT3cwvksRhXaA5LfVH0oB3XGNhrOzteP66UquAJUaNRKnMRjH4VjBiw9EWD1YGImGzeFPpA0p2xTKXQZAfLalNGnDRXM+3BlUfJsFbaSR+Uu/C3MIb8PMyA6h1nQGxMaIZLnl/Py2LPFgo5prafgdcD+9tV/BqE9F89AJC5LvHwOSKTfvsF9qe0fsZHFjg/+h10hZdeF8b1bQBhVO2OZf/T/e94I85MOh";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    boolean positionSet=false;
    boolean started=false;
    boolean waitToDrive=false;
    long timer=0;

    private static int numRings=0;
    public SampleMecanumDrive drive;

    //Set A Trajectories
    Trajectory t_A_0, t_A_1, t_A_1a, t_A_2, t_A_3, t_A_4;
    //Set B Trajectories
    Trajectory t_B_0, t_B_1, t_B_2, t_B_3, t_B_4;
    //Set C Trajectories
    Trajectory t_C_0, t_C_1, t_C_2, t_C_3, t_C_4;

    enum State {
        t0,
        t1,
        t1a,
        t2,
        t3,
        t4,
        firstWobbleDrop,
        secondWobbleDrop,
        wobbleGrab,
        shoot,
        IDLE
    }
    enum Set {
        A,
        B,
        C,
        UNKNOWN
    }

    State currentState = State.IDLE;
    Set set = Set.UNKNOWN;
    Pose2d startPose = new Pose2d(-58, -48, 0);

    @Override
    public void init() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        initTrajectories();

        currentState=State.t0;
    }

    @Override
    public void init_loop(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null && updatedRecognitions.size()>0) {
                if(updatedRecognitions.get(0).getLabel().equals("Quad")) numRings=4;
                else if(updatedRecognitions.get(0).getLabel().equals("Single")) numRings=1;
            }else {
                numRings=0;
            }
        }
        telemetry.addData("Num Rings: ",numRings);
        telemetry.update();
        if(numRings==0) set = Set.A;
        else if(numRings==1) set = Set.B;
        else if(numRings==4) set = Set.C;
    }

    @Override
    public void loop() {
        switch(currentState){
            case t0:
                if(!drive.isBusy() && !started){
                    switch(set){
                        case A:
                            drive.followTrajectoryAsync(t_A_0);
                            break;
                        case B:
                            drive.followTrajectoryAsync(t_B_0);
                            break;
                        case C:
                            drive.followTrajectoryAsync(t_C_0);
                            break;
                    }
                    started=true;
                }else if(!drive.isBusy()){//after first trajectory
                    currentState = State.firstWobbleDrop;
                    positionSet=false;
                    timer=System.currentTimeMillis();
                }break;
            case firstWobbleDrop:
                if(System.currentTimeMillis()-timer>4000){
                    currentState = State.t1;
                    switch(set){
                        case A:
                            drive.followTrajectoryAsync(t_A_1);
                            break;
                        case B:
                            drive.followTrajectoryAsync(t_B_1);
                            break;
                        case C:
                            drive.followTrajectoryAsync(t_C_1);
                            break;
                    }
                }break;
            case t1:
                if(!drive.isBusy()){//after second trajectory
                    currentState = State.t1a;
                    switch(set){
                        case A:
                            drive.followTrajectoryAsync(t_A_1a);
                            break;
                        case B:
                            drive.followTrajectoryAsync(t_A_1a);
                            break;
                        case C:
                            drive.followTrajectoryAsync(t_A_1a);
                            break;
                    }
                }break;
            case t1a:
                if(!drive.isBusy()){//after second trajectory
                    currentState = State.wobbleGrab;
                    positionSet=false;
                    timer = System.currentTimeMillis();
                }break;
            case wobbleGrab:
                if(System.currentTimeMillis()-timer>4000) {
                    currentState = State.t2;
                    switch(set){
                        case A:
                            drive.followTrajectoryAsync(t_A_2);
                            break;
                        case B:
                            drive.followTrajectoryAsync(t_B_2);
                            break;
                        case C:
                            drive.followTrajectoryAsync(t_C_2);
                            break;
                    }
                }break;
            case t2:
                if(!drive.isBusy()){//after third trajectory
                    currentState = State.secondWobbleDrop;
                    positionSet=false;
                    timer = System.currentTimeMillis();
                }break;
            case secondWobbleDrop:
                if(System.currentTimeMillis()-timer>4000){
                    currentState = State.t3;
                    switch(set){
                        case A:
                            drive.followTrajectoryAsync(t_A_3);
                            break;
                        case B:
                            drive.followTrajectoryAsync(t_B_3);
                            break;
                        case C:
                            drive.followTrajectoryAsync(t_C_3);
                            break;
                    }
                }break;
            case t3:
                if(!drive.isBusy()){//after fourth trajectory
                    currentState = State.t4;
                    switch(set){
                        case A:
                            drive.followTrajectoryAsync(t_A_4);
                            break;
                        case B:
                            drive.followTrajectoryAsync(t_B_4);
                            break;
                        case C:
                            drive.followTrajectoryAsync(t_C_4);
                            break;
                    }
                }break;
            case t4:
                if(!drive.isBusy()){
                    currentState = State.IDLE;
                }break;
            case IDLE:
                break;
        }
        drive.update();
        if(currentState==State.wobbleGrab){
            pickupWobbleGoal(timer);
        }else if(currentState==State.secondWobbleDrop || currentState==State.firstWobbleDrop){
            dropWobbleGoal(timer);
        } else if(currentState==State.shoot){
            shootRings(timer);
        }
    }

    public void initTrajectories() {
        //------------------------ 0 Rings -> Zone A -> Set A ------------------------
        //Move from starting position to zone A with preloaded Wobble Goal
        t_A_0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0,-56),0)
                .build();
        //Drop Wobble Goal
        //Move back to other wobble goal
        t_A_1 = drive.trajectoryBuilder(t_A_0.end(), true)
                .splineToLinearHeading(new Pose2d(-40,-50, Math.toRadians(90)),0)
                .build();
        t_A_1a = drive.trajectoryBuilder(t_A_1.end(), true)
                .lineTo(new Vector2d(-40,-35))
                .build();
        //Pick Up Wobble Goal
        //Move back to zone A with grabbed Wobble Goal
        t_A_2 = drive.trajectoryBuilder(t_A_1.end())
                .splineTo(new Vector2d(-8,-60),0)
                .build();
        //Drop second wobble goal
        //Move to shooting line, lined up with goal
        t_A_3 = drive.trajectoryBuilder(t_A_2.end())
                .splineToConstantHeading(new Vector2d(0,-36),0)
                .build();
        //Move to parking line
        //Shoot Rings
        t_A_4 = drive.trajectoryBuilder(t_A_3.end())
                .lineTo(new Vector2d(16, -36))
                .build();

        //------------------------ 1 Ring -> Zone B -> Set B ------------------------
        //Move from starting position to zone A with preloaded Wobble Goal
        t_B_0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-24,-60),0)
                .splineTo(new Vector2d(36,-36),0)
                .build();
        //Drop Wobble Goal
        //Move back to other wobble goal
        t_B_1 = drive.trajectoryBuilder(t_B_0.end(), true)
                .splineTo(new Vector2d(-24,-60),0)
                .splineTo(new Vector2d(-48,-48),Math.toRadians(90))
                .build();
        //Pick Up Wobble Goal
        //Move back to zone B with grabbed Wobble Goal
        t_B_2 = drive.trajectoryBuilder(t_B_1.end())
                .splineTo(new Vector2d(-24,-60),0)
                .splineTo(new Vector2d(24,-36),0)
                .build();
        //Drop second wobble goal
        //Move to shooting line, lined up with goal
        t_B_3 = drive.trajectoryBuilder(t_B_2.end())
                .splineTo(new Vector2d(0,-36),0)
                .build();
        //Shoot Rings
        //Move to parking line
        t_B_4 = drive.trajectoryBuilder(t_A_3.end())
                .lineTo(new Vector2d(10, -36))
                .build();

        //------------------------ 4 Rings -> Zone C -> Set C ------------------------
        //Move from starting position to zone A with preloaded Wobble Goal
        t_C_0 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(60,-60),0)
                .build();
        //Drop Wobble Goal
        //Move back to other wobble goal
        t_C_1 = drive.trajectoryBuilder(t_C_0.end(), true)
                .splineTo(new Vector2d(-48,-48),Math.toRadians(90))
                .build();
        //Pick Up Wobble Goal
        //Move back to zone C with grabbed Wobble Goal
        t_C_2 = drive.trajectoryBuilder(t_C_1.end())
                .splineTo(new Vector2d(48,-60),0)
                .build();
        //Drop second wobble goal
        //Move to shooting line, lined up with goal
        t_C_3 = drive.trajectoryBuilder(t_C_2.end())
                .splineTo(new Vector2d(0,-36),0)
                .build();
        //Shoot Rings
        //Move to parking line
        t_C_4 = drive.trajectoryBuilder(t_A_3.end())
                .lineTo(new Vector2d(10, -36))
                .build();
    }

    public void shootRings(long timer){
        drive.shooterOn();
        drive.intakeOn();
        while(System.currentTimeMillis()-timer<5000){}
        drive.toggleShooter();
        drive.toggleIntake();
    }
    public void shootOneRing(){
        drive.shooterOn();
        drive.intakeOn();
        while(System.currentTimeMillis()-timer<1000){}
        drive.toggleShooter();
        drive.toggleIntake();
    }
    public void dropWobbleGoal(long timer){
        if(System.currentTimeMillis()-timer<1000 && !positionSet) {
            drive.setWobbleGrabberArmPosition(-700);
            positionSet=true;
        }
        if(System.currentTimeMillis()-timer>1000 && drive.wobbleGrabber.getPosition()==1){
            releaseWobble();
            positionSet=false;
        }if(System.currentTimeMillis()-timer>2000 && !positionSet && currentState==State.secondWobbleDrop){
            drive.setWobbleGrabberArmPosition(-70);
            positionSet=true;
        }
    }
    public void pickupWobbleGoal(long timer){
        if(System.currentTimeMillis()-timer<1000 && !positionSet){
            drive.setWobbleGrabberArmPosition(-700);
            positionSet=true;
        }
        if(System.currentTimeMillis()-timer>1000 && drive.wobbleGrabber.getPosition()==0) {
            grabWobble();
            positionSet=false;
        }if(System.currentTimeMillis()-timer>2000 && !positionSet){
            drive.setWobbleGrabberArmPosition(-70);
            positionSet=true;
        }
    }
    public void releaseWobble(){
        drive.wobbleGrabber.setPosition(0);
    }
    public void grabWobble(){
        drive.wobbleGrabber.setPosition(1);
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}