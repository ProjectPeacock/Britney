package org.firstinspires.ftc.teamcode.Libs;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile;

public class AutoParams {
    /*

    PARAMETER MAP - PLEASE READ BEFORE EDITING

    This file contains all the parameters for BlueTerminalAuto and RedTerminalAuto.
    This file also contains all Lift encoder values for cone retrieval from the stack.

    If making a new auto, include "AutoParams params = new AutoParams();" within the class.

    All headings are converted to radians here, so there is no need to convert them in the main autonomous programs.

    All values are based on BlueTerminalAuto and are negated as needed in RedTerminalAuto
     */
    HWProfile robot = new HWProfile();

    //number of cycles
    public final int numMidCycles=3;
    public final int numHighCycles=0;
    //claw open time
    public final double timeOpen=0.35;

    //claw close time
    public final double timeClose=0.35;

    //lift power
    public final double liftPow=1;

    public final int liftTicksPerInch=38;

    //lift heights for grabbing cones from stack
    public final int cycle1=(int)12.0*liftTicksPerInch;
    public final int cycle2=(int)11.0*liftTicksPerInch;
    public final int cycle3=(int)9.0*liftTicksPerInch;
    public final int cycle4=(int)7.0*liftTicksPerInch;
    public final int cycle5=(int)0;

    //amount of difference between lift motor targets
    public final int diffConstant = 20;

    //staring pose X and Y
    public final double startPoseX=38.25;
    public final double startPoseCRIBlueX=12;
    public final double startPoseY=-63;
    public final double startPoseCRIBlueY=-88;

//preload parameters
    //scoring position for preload on mid pole X, Y, and Heading
    public final double preloadMidX=28.5;
    public final double preloadMidY=-34;
    public final double preloadMidHeading=Math.toRadians(120);

    //amount to drive forward for preload approach
    public final double preloadMidForward=4;

    //amount to drive backward for preload retreat
    public final double preloadMidBackward=8;

    //amount to turn to reorient to original heading after scoring preload
    public final double preloadReorientHeading=Math.toRadians(-40);

    //waypoint to cone stack X, Y, and Heading
    public final double preloadWaypointX=45;
    public final double preloadWaypointY=-12;
    public final double preloadWaypointHeading=Math.toRadians(0);

    //forward distance to align with cone stack (1st cycle only)
    public final double cycle1coneStackForward=14.5;

//mid pole cycle parameters
    //reverse dist from cone stack
    public final double coneStackReverse=12;

    //mid pole cycle scoring position X, Y, and Heading
    public final double cycleMidX=24;
    public final double cycleMidY=-10;
    public final double cycleMidHeading=Math.toRadians(270);

    //distance to drive forward to align to pole in mid cycle
    public final double cycleMidForward=4;

    //distance to reverse from pole in mid cycle
    public final double cycleMidReverse=4;

    //cone retrieval position X, Y, and Heading
    public final double coneStackAlignX=56;
    public final double coneStackAlignY=-12;
    public final double coneStackAlignHeadingBlue=Math.toRadians(0);
    public final double coneStackAlignHeadingRed=Math.toRadians(180);

    //approach dist to cone stack
    public final double coneStackForward=3.5;

//high pole cycle parameters
    //X, Y, and Heading of mid pole cycle scoring position
    public final double cycleHighX=24;
    public final double cycleHighY=-14;
    public final double cycleHighHeading=Math.toRadians(90);

    //distance to drive forward to align to pole in mid cycle
    public final double cycleHighForward=4;

    //distance to reverse from pole in mid cycle
    public final double cycleHighReverse=4;

//parking parameters
    //12 inch park strafe (pos 1 on Blue, pos 3 on Red)
    public final double park12Inch=12;

    //36 inch park strafe (pos 3 on Blue, pos 1 on Red)
    public final double park36Inch=36;

}
