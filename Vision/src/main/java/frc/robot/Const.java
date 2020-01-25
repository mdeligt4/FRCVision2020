package frc.robot;

import org.opencv.core.Scalar;

public class Const {
  // double H = 61/2, tH = 20;
    // double S = 11.8*2.55, tS = 15;
    // double V = 92.5*2.55, tV = 15;
    // Scalar lowerb = new Scalar(H-tH,S-tS,V-tV);
    // Scalar upperb = new Scalar(H+tH,S+tS,V+tV);
  public static double lH = 50 / 2;
  public static double lS = 30 * 2.55;
  public static double lV = 25 * 2.55;
  public static double hH = 110 / 2;
  public static double hS = 70 * 2.55;
  public static double hV = 70 * 2.55;
  public static Scalar lowerb = new Scalar(lH, lS, lV);
  public static Scalar upperb = new Scalar(hH, hS, hV);
  public static Scalar iwanttodie = new Scalar(255, 255, 255);
  public static double minSolidity = 0.15, maxSolidity = 0.2;
  public static double minAspectRatio = 0.020, maxAspectRatio = 0.028;
}