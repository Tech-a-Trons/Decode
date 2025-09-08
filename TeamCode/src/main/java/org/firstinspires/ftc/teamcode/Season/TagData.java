package org.firstinspires.ftc.teamcode.Season;

//Extracts Apriltag Data

public class TagData {

    public int id;
    public double tx;
    public double ty;
    public double tz;
    public boolean valid;

    public TagData(int id, double tx, double ty, double tz, boolean valid) {
        this.id = id;
        this.tx = tx;
        this.ty = ty;
        this.tz = tz;
        this.valid = valid;
    }
}


