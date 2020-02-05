package frc.robot.math;

public class SquareMatrix extends Matrix {
	int size;

	public SquareMatrix(int size) {
		super(size, size);
		this.size = size;
		// TODO Auto-generated constructor stub
	}

	public SquareMatrix scale(double scale) {
		super.scale(scale);
		return this;
	}

	public static SquareMatrix multiply(Matrix a, Matrix b) {
		if (a.xSize != b.ySize && a.ySize != b.xSize) {
			return null;
		}
		SquareMatrix result = new SquareMatrix(a.xSize);
		for (int x = 0; x < result.size; x++) {
			for (int y = 0; y < result.size; y++) {
				for (int z = 0; z < a.xSize; z++) {
					result.getMat()[x][y] += a.getMat()[x][z] * b.getMat()[z][y];
				}
			}
		}
		return result;
	}

	public SquareMatrix invert() {
//		double d = 1;
		for (int p = 0; p < size; p++) {
			if (getMat()[p][p] == 0) {
				return null;
			}
//			d *= mat[p][p];
			for (int y = 0; y < size; y++) {
				if (y == p) {
					continue;
				}
				getMat()[p][y] /= getMat()[p][p];
			}
			for (int x = 0; x < size; x++) {
				if (x == p) {
					continue;
				}
				getMat()[x][p] /= -getMat()[p][p];
			}
			for (int x = 0; x < size; x++) {
				if (x == p) {
					continue;
				}
				for (int y = 0; y < size; y++) {
					if (y == p) {
						continue;
					}
					getMat()[x][y] += getMat()[x][p] * getMat()[p][y] * getMat()[p][p];
				}
			}
			getMat()[p][p] = 1 / getMat()[p][p];

		}
		return this;
	}

	public static SquareMatrix identity(int size) {
		SquareMatrix identity = new SquareMatrix(size);
		for (int i = 0; i < size; i++) {
			identity.getMat()[i][i] = 1;
		}
		return identity;
	}

	public static SquareMatrix add(SquareMatrix a, SquareMatrix b) {
		if (a.size != b.size) {
			return null;
		}
		SquareMatrix result = new SquareMatrix(a.size);
		for (int x = 0; x < result.size; x++) {
			for (int y = 0; y < result.size; y++) {
				result.getMat()[x][y] = a.getMat()[x][y] + b.getMat()[x][y];
			}
		}
		return result;
	}

	public SquareMatrix transpose() {
		SquareMatrix transposed = new SquareMatrix(size);
		for (int x = 0; x < xSize; x++) {
			for (int y = 0; y < ySize; y++) {
				transposed.getMat()[y][x] = getMat()[x][y];
			}
		}
		return transposed;
	}

	public SquareMatrix clone() {
		SquareMatrix clone = new SquareMatrix(size);
		for (int x = 0; x < size; x++) {
			for (int y = 0; y < size; y++) {
				clone.getMat()[x][y] = getMat()[x][y];
			}
		}
		return clone;
	}

}
