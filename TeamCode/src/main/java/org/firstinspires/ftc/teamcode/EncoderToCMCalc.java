package org.firstinspires.ftc.teamcode;

public class EncoderToCMCalc
{
    static float DIAMETER_CM = 10;
    static float COUNTS_PER_REV = 1120;
    static float PI = (float)Math.PI;
    static float CONVERSION_CONSTANT = (float)(1/2.45);
    static float LR_PER_REV_IN = (float)14.5;

    static float COUNTS_PER_FB_CM = COUNTS_PER_REV / (DIAMETER_CM * PI);
    static float COUNTS_PER_LR_CM = COUNTS_PER_REV / (LR_PER_REV_IN * (float)2.45);

    public float Forward_Backwards(boolean isInCM)
    {
        if(isInCM) { return COUNTS_PER_FB_CM; } //returns in CM
        else { return COUNTS_PER_FB_CM * CONVERSION_CONSTANT; } //returns in IN
    }

    public float Left_Right(boolean isInCM)
    {
        if(isInCM) { return COUNTS_PER_LR_CM; } //returns in CM
        else { return COUNTS_PER_LR_CM * CONVERSION_CONSTANT; } //returns in IN
    }

    public float Zero()
    {
        return 0;
    }
}
