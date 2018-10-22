package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRoverDepot", group = "Main")
public class AutonomousRoverDepot extends AutonomousRover{
    @Override
    protected boolean depotSide() {
        return true;
    }


}
