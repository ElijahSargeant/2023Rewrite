package frc.robot.enums;

import com.pathplanner.lib.PathConstraints;


public enum Autos {
    
    ONE_PIECE_LEAVE     ("OnePieceLeave",    new PathConstraints(3.5, 3.5)),
    TWO_PIECE_NO_BUMP   ("TwoPieceNoBump",   new PathConstraints(3.5, 3.5)),
    TWO_PIECE_BUMP      ("TwoPieceBump",     new PathConstraints(3.5, 3.5)),
    THREE_PIECE_NO_BUMP ("ThreePieceNoBump", new PathConstraints(3.5, 3.5));

    private final String autoString;
    private final PathConstraints constraints;

    Autos(String autoString, PathConstraints constraints) {
        this.autoString = autoString;
        this.constraints = constraints;
    }

    public String path() {return autoString;}
    public PathConstraints constraints() {return constraints;}

}
