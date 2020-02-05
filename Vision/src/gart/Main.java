package gart;

import java.util.Arrays;

import gart.math.Coordinates2D;
import gart.math.Coordinates3D;
import gart.vision.CameraSolver;

public class Main {
	// DLI DRI ULI URI
	// DLO DRO ULO URO
	public static void main(String[] args) {
		CameraSolver sol = new CameraSolver(new Coordinates3D(-8.66, -15, 0), new Coordinates3D(8.66, -15, 0),
				new Coordinates3D(-17.32, 0, 0), new Coordinates3D(17.32, 0, 0));

		sol.solve(new Coordinates2D[] { new Coordinates2D(-25.01, 172.28), new Coordinates2D(-17.81, 196.54),
				new Coordinates2D(-65.04, 156.34), new Coordinates2D(-47.47, 204.4) });

//		CameraSolver sol = new CameraSolver(new Coordinates3D(-9.81, -17, 0), new Coordinates3D(9.81, -17, 0),
//				new Coordinates3D(-19.63, 0, 0), new Coordinates3D(19.63, 0, 0));

//		sol.solve(new Coordinates2D[] { new Coordinates2D(129, -76), new Coordinates2D(147, 39),
//				new Coordinates2D(32, -139), new Coordinates2D(59, 111) });
		
		// CameraSolver.heapPermutation(new int[] { 1, 2, 3, 4, 5, 6, 7, 8 });
//		generateSublist(new int[] { 1, 2, 3, 4, 5, 6 }, 3);
	}

	public static void generateSublist(int[] list, int k) {
		generatePermutation(list, k, 0, new int[k]);
	}

	public static void generatePermutation(int[] list, int k, int lowbound, int[] result) {
		if (k == 0) {
			System.out.println(Arrays.toString(result));
			return;
		}
		for (int i = lowbound; i < list.length - k + 1; i++) {
			result[k - 1] = list[i];
			generatePermutation(list, k - 1, i + 1, result);
		}
	}
}
