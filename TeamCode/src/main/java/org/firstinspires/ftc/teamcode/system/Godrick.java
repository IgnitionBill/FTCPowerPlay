package org.firstinspires.ftc.teamcode.system;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

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
    public MotionSequenceDirector motionSequenceDirector = new MotionSequenceDirector();
    public MechanumController mechanumController = new MechanumController();
    public Arm arm = new Arm();
    public GamePadState gamePadState = new GamePadState();
    public Actuators actuators = new Actuators();
    public Sensors sensors = new Sensors();
    public ArmController armController = new ArmController();
    public ArmPoseGenerator definedArmPositions = new ArmPoseGenerator();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    private static final Godrick instance = new Godrick();

    // private constructor to avoid client applications using the constructor
    private Godrick(){}

    public static Godrick getInstance() {
        return instance;
    }

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        Godrick r = getInstance();
        r.hardwareMap = hardwareMap;
        r.telemetry = telemetry;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        r.actuators.initializeGodrick(hardwareMap, telemetry);
        r.sensors.initialize(hardwareMap, telemetry);
        r.mechanumController.initialize(telemetry);
        r.armController.initialize();
        r.gamePadState.initialize(telemetry);
    }

    public void update(){
        telemetry.addData("Alt mode:", gamePadState.altMode);
        // store the latest gamepad state
        gamePadState.update(gamepad1, false);

        // update the sensors with data from the actuators
        sensors.update(actuators, true);

        // update the arm
        armController.updateArm(false);

        // update the drivetrain
        mechanumController.simpleMechanumUpdate(gamePadState, sensors, false);

        // update the actuators
        actuators.updateDrivetrainMotors(mechanumController);
        actuators.updateArm(armController, sensors);

        // display all telemetry updates to the controller, use verbose=true to see reports in telemetry
        telemetry.update();
    }

}
