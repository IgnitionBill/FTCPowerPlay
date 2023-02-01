package org.firstinspires.ftc.teamcode.system;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.util.Log;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.ArmController;
import org.firstinspires.ftc.teamcode.arm.ArmPoseGenerator;
import org.firstinspires.ftc.teamcode.drivetrain.MechanumController;
import org.firstinspires.ftc.teamcode.sequence.MotionSequenceDirector;

/**
 * The robot holds convenient references to everything.
 */
public class Godrick {
//    INSTANCE;

    public MotionSequenceDirector motionSequenceDirector;
    public MechanumController mechanumController;
    public Arm arm;
    public GamePadState gamePadState;
    public Actuators actuators;
    public Sensors sensors;
    public ArmController armController;
    public ArmPoseGenerator definedArmPositions;

    public Gamepad gamepad1;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    //private static final Godrick instance = new Godrick();

//     private constructor to avoid client applications using the constructor
    public Godrick(){
        Log.d("Godrick", "Constructing");
        try {
            motionSequenceDirector = new MotionSequenceDirector();
            mechanumController = new MechanumController();
            arm = new Arm();
            sensors = new Sensors();
            actuators = new Actuators();
            gamePadState = new GamePadState();
            armController = new ArmController();
            definedArmPositions = new ArmPoseGenerator();
        }
        catch (Exception e){
            Log.e("Godrick", e.toString());
            return;
        }
        Log.d("Godrick", "Construction Completed");
    }

//    public static Godrick getInstance() {
//        return instance;
//    }

    public void initialize(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry){
        try{
            Log.e("Godrick", "Initializing");
            this.hardwareMap = hardwareMap;
            this.telemetry = telemetry;
            this.gamepad1 = gamepad1;

            // JUST STRUGGLING TO SURVIVE...
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            actuators.initializeGodrick(this);
            sensors.initialize(this);
            mechanumController.initialize(telemetry);
            armController.initialize(this);
            gamePadState.initialize(telemetry);
            motionSequenceDirector.initialize(this);
        }
        catch (Exception e){
            Log.e("Godrick", e.toString());
            return;
        }
        Log.e("Godrick", "Initialize Completed");
    }

    public void update(){
        telemetry.addData("Alt mode:", gamePadState.altMode);
        // store the latest gamepad state
        gamePadState.update(gamepad1, false);

        // update the sensors with data from the actuators
        sensors.update(actuators, true);

        // update the arm
        armController.updateArm(true);

        // update the drivetrain
        mechanumController.simpleMechanumUpdate(gamePadState, sensors, false);

        // update the actuators
        actuators.updateDrivetrainMotors(mechanumController);
        actuators.updateArm(armController, sensors);

        // display all telemetry updates to the controller, use verbose=true to see reports in telemetry
        telemetry.update();
    }

}
