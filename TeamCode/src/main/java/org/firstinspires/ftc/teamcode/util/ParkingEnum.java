package org.firstinspires.ftc.teamcode.util;

public enum ParkingEnum {
    PARK1("1p"), PARK2("2p"), PARK3("3p");

    private final String label;

    ParkingEnum(String s) {
        label = s;
    }

    public String getLabel() {
        return label;
    }
}
