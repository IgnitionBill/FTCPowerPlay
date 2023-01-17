package org.firstinspires.ftc.teamcode.arm;

import java.util.ArrayList;

public class DefinedMotionSequences {

    // Arm sequences
    public MotionSequence foldedToCarry = new MotionSequence();

    public MotionSequence upToCarry = new MotionSequence();
    public MotionSequence carryToUp = new MotionSequence();

    public MotionSequence carryToFold = new MotionSequence();
    public MotionSequence carryToFar = new MotionSequence();
    public MotionSequence carryToClose = new MotionSequence();
    public MotionSequence carryToHigh = new MotionSequence();
    public MotionSequence carryToMiddle = new MotionSequence();
    public MotionSequence carryToLow = new MotionSequence();

    public MotionSequence farToFolded = new MotionSequence();
    public MotionSequence farToCarry = new MotionSequence();
    public MotionSequence farToClose = new MotionSequence();
    public MotionSequence farToHigh = new MotionSequence();
    public MotionSequence farToMiddle = new MotionSequence();
    public MotionSequence farToLow = new MotionSequence();

    public MotionSequence closeToFolded = new MotionSequence();
    public MotionSequence closeToCarry = new MotionSequence();
    public MotionSequence closeToFar = new MotionSequence();
    public MotionSequence closeToHigh = new MotionSequence();
    public MotionSequence closeToMiddle = new MotionSequence();
    public MotionSequence closeToLow = new MotionSequence();

    public MotionSequence highToFold = new MotionSequence();
    public MotionSequence highToCarry = new MotionSequence();
    public MotionSequence highToMiddle = new MotionSequence();
    public MotionSequence highToLow = new MotionSequence();
    public MotionSequence highToFar = new MotionSequence();
    public MotionSequence highToClose = new MotionSequence();

    public MotionSequence middleToFolded = new MotionSequence();
    public MotionSequence middleToCarry = new MotionSequence();
    public MotionSequence middleToHigh = new MotionSequence();
    public MotionSequence middleToLow = new MotionSequence();
    public MotionSequence middleToFar = new MotionSequence();
    public MotionSequence middleToClose = new MotionSequence();

    public MotionSequence lowToFolded = new MotionSequence();
    public MotionSequence lowToCarry = new MotionSequence();
    public MotionSequence lowToHigh = new MotionSequence();
    public MotionSequence lowToMiddle = new MotionSequence();
    public MotionSequence lowToFar = new MotionSequence();
    public MotionSequence lowToClose = new MotionSequence();

    public void init(DefinedArmPositions definedArmPositions){

        foldedToCarry.add(definedArmPositions.carryPosition);

        //--------------------------------- 4 testing
        carryToUp.add(definedArmPositions.straightUpPosition);
        upToCarry.add(definedArmPositions.carryPosition);

        //--------------------------------- Carry to other

        carryToFold.add(definedArmPositions.foldedPosition);
        // Initialize the current sequence to carryToFolded, and record that its done
        //currentSequence = carryToFold;

        //carryToFar.add(grabTransition);
        carryToFar.add(definedArmPositions.farPosition);

        //carryToClose.add(grabTransition);
        carryToClose.add(definedArmPositions.closePosition);

        //carryToHigh.add(placeTransition);
        carryToHigh.add(definedArmPositions.highPosition);

        //carryToMiddle.add(placeTransition);
        carryToMiddle.add(definedArmPositions.middlePosition);

        //carryToLow.add(placeTransition);
        carryToLow.add(definedArmPositions.lowPosition);

        //--------------------------------- Far to other
        
        //farToFolded.add(carryTransition);
        farToFolded.add(definedArmPositions.foldedPosition);

        //farToCarry.add(carryTransition);
        farToCarry.add(definedArmPositions.carryPosition);

        //farToClose.add(grabTransition);
        farToClose.add(definedArmPositions.closePosition);

        //farToHigh.add(placeTransition);
        farToHigh.add(definedArmPositions.highPosition);

        //farToMiddle.add(placeTransition);
        farToMiddle.add(definedArmPositions.middlePosition);

        //farToLow.add(placeTransition);
        farToLow.add(definedArmPositions.lowPosition);

        //--------------------------------- Close to other

        //closeToFolded.add(carryTransition);
        closeToFolded.add(definedArmPositions.foldedPosition);

        //closeToCarry.add(carryTransition);
        closeToCarry.add(definedArmPositions.carryPosition);

        //closeToFar.add(grabTransition);
        closeToFar.add(definedArmPositions.farPosition);

        //closeToHigh.add(placeTransition);
        closeToHigh.add(definedArmPositions.highPosition);

        //closeToMiddle.add(placeTransition);
        closeToMiddle.add(definedArmPositions.middlePosition);

        //closeToLow.add(placeTransition);
        closeToLow.add(definedArmPositions.lowPosition);

        //--------------------------------- High to other

        //highToFold.add(carryTransition);
        highToFold.add(definedArmPositions.foldedPosition);

        //highToCarry.add(carryTransition);
        highToCarry.add(definedArmPositions.carryPosition);

        //highToFar.add(grabTransition);
        highToFar.add(definedArmPositions.farPosition);

        //highToClose.add(grabTransition);
        highToClose.add(definedArmPositions.closePosition);

        //highToMiddle.add(placeTransition);
        highToMiddle.add(definedArmPositions.middlePosition);

        //highToLow.add(placeTransition);
        highToLow.add(definedArmPositions.lowPosition);

        //--------------------------------- Middle to other

        //middleToFolded.add(carryTransition);
        middleToFolded.add(definedArmPositions.foldedPosition);

        //middleToCarry.add(carryTransition);
        middleToCarry.add(definedArmPositions.carryPosition);

        //middleToFar.add(grabTransition);
        middleToFar.add(definedArmPositions.farPosition);

        //middleToClose.add(grabTransition);
        middleToClose.add(definedArmPositions.closePosition);

        //middleToHigh.add(placeTransition);
        middleToHigh.add(definedArmPositions.highPosition);

        //middleToLow.add(placeTransition);
        middleToLow.add(definedArmPositions.lowPosition);

        //--------------------------------- Low to other

        //lowToFolded.add(carryTransition);
        lowToFolded.add(definedArmPositions.foldedPosition);

        //lowToCarry.add(carryTransition);
        lowToCarry.add(definedArmPositions.carryPosition);

        //lowToFar.add(grabTransition);
        lowToFar.add(definedArmPositions.farPosition);

        //lowToClose.add(grabTransition);
        lowToClose.add(definedArmPositions.closePosition);

        //lowToHigh.add(placeTransition);
        lowToHigh.add(definedArmPositions.highPosition);

        //lowToMiddle.add(placeTransition);
        lowToMiddle.add(definedArmPositions.middlePosition);

    }
}
