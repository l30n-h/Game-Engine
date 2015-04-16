package com.brocorporation.gameengine.utils;

public class Vector3f {

	public final static Vector3f UP = new Vector3f(0, 1, 0);

	public float x;
	public float y;
	public float z;

	public Vector3f() {
		this(0, 0, 0);
	}

	public Vector3f(final float pX, final float pY, final float pZ) {
		set(pX, pY, pZ);
	}

	public Vector3f(final Vector3f vector) {
		this(vector.x, vector.y, vector.z);
	}

	public Vector3f set(final float pX, final float pY, final float pZ) {
		x = pX;
		y = pY;
		z = pZ;
		return this;
	}

	public Vector3f set(final Vector3f vector) {
		return set(vector.x, vector.y, vector.z);
	}

	public Vector3f abs() {
		return set(Math.abs(x), Math.abs(y), Math.abs(z));
	}

	public Vector3f invert() {
		return set(-x, -y, -z);
	}

	public Vector3f setInvert(final Vector3f vector) {
		return set(-vector.x, -vector.y, -vector.z);
	}

	public Vector3f add(final float pX, final float pY, final float pZ) {
		return set(x + pX, y + pY, z + pZ);
	}

	public Vector3f add(final Vector3f vector) {
		return add(vector.x, vector.y, vector.z);
	}

	public Vector3f setAdd(final float pX1, final float pY1, final float pZ1,
			final float pX2, final float pY2, final float pZ2) {
		return set(pX1 + pX2, pY1 + pY2, pZ1 + pZ2);
	}

	public Vector3f setAdd(final Vector3f vector1, final Vector3f vector2) {
		return setAdd(vector1.x, vector1.y, vector1.z, vector2.x, vector2.y,
				vector2.z);
	}

	public Vector3f addScaled(final float pX, final float pY, final float pZ,
			final float scalar) {
		return add(pX * scalar, pY * scalar, pZ * scalar);
	}

	public Vector3f addScaled(final Vector3f vector, final float scalar) {
		return addScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Vector3f setAddScaled(final float pX1, final float pY1,
			final float pZ1, final float pX2, final float pY2, final float pZ2,
			final float scalar) {
		return set(pX1 + pX2 * scalar, pY1 + pY2 * scalar, pZ1 + pZ2 * scalar);
	}

	public Vector3f setAddScaled(final Vector3f vector1,
			final Vector3f vector2, final float scalar) {
		return setAddScaled(vector1.x, vector1.y, vector1.z, vector2.x,
				vector2.y, vector2.z, scalar);
	}

	public Vector3f subtract(final float pX, final float pY, final float pZ) {
		return set(x - pX, y - pY, z - pZ);
	}

	public Vector3f subtract(final Vector3f vector) {
		return subtract(vector.x, vector.y, vector.z);
	}

	public Vector3f setSubtract(final float pX1, final float pY1,
			final float pZ1, final float pX2, final float pY2, final float pZ2) {
		return set(pX1 - pX2, pY1 - pY2, pZ1 - pZ2);
	}

	public Vector3f setSubtract(final Vector3f vector1, final Vector3f vector2) {
		return setSubtract(vector1.x, vector1.y, vector1.z, vector2.x,
				vector2.y, vector2.z);
	}

	public Vector3f subtractScaled(final float pX, final float pY,
			final float pZ, final float scalar) {
		return subtract(pX * scalar, pY * scalar, pZ * scalar);
	}

	public Vector3f subtractScaled(final Vector3f vector, final float scalar) {
		return subtractScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Vector3f setSubtractScaled(final float pX1, final float pY1,
			final float pZ1, final float pX2, final float pY2, final float pZ2,
			final float scalar) {
		return set(pX1 - pX2 * scalar, pY1 - pY2 * scalar, pZ1 - pZ2 * scalar);
	}

	public Vector3f setSubtractScaled(final Vector3f vector1,
			final Vector3f vector2, final float scalar) {
		return setSubtractScaled(vector1.x, vector1.y, vector1.z, vector2.x,
				vector2.y, vector2.z, scalar);
	}

	public Vector3f multiplyVM3(final Vector3f vector, float[] rhsMat,
			int rhsMatOffset) {
		return set(x = rhsMat[rhsMatOffset] * vector.x
				+ rhsMat[rhsMatOffset + 1] * vector.y
				+ rhsMat[rhsMatOffset + 2] * vector.z,
				y = rhsMat[rhsMatOffset + 3] * vector.x
						+ rhsMat[rhsMatOffset + 4] * vector.y
						+ rhsMat[rhsMatOffset + 5] * vector.z,
				z = rhsMat[rhsMatOffset + 6] * vector.x
						+ rhsMat[rhsMatOffset + 7] * vector.y
						+ rhsMat[rhsMatOffset + 8] * vector.z);
	}

	public Vector3f multiplyM3V(float[] lhsMat, int lhsMatOffset,
			Vector3f vector) {
		return set(vector.x * lhsMat[lhsMatOffset] + vector.y
				* lhsMat[lhsMatOffset + 3] + vector.z
				* lhsMat[lhsMatOffset + 6], vector.x * lhsMat[lhsMatOffset + 1]
				+ vector.y * lhsMat[lhsMatOffset + 4] + vector.z
				* lhsMat[lhsMatOffset + 7], vector.x * lhsMat[lhsMatOffset + 2]
				+ vector.y * lhsMat[lhsMatOffset + 5] + vector.z
				* lhsMat[lhsMatOffset + 8]);
	}

	public Vector3f multiplyVM4(final Vector3f vector, float[] rhsMat,
			int rhsMatOffset) {
		return set(rhsMat[rhsMatOffset] * vector.x + rhsMat[rhsMatOffset + 1]
				* vector.y + rhsMat[rhsMatOffset + 2] * vector.z,
				rhsMat[rhsMatOffset + 4] * vector.x + rhsMat[rhsMatOffset + 5]
						* vector.y + rhsMat[rhsMatOffset + 6] * vector.z,
				rhsMat[rhsMatOffset + 8] * vector.x + rhsMat[rhsMatOffset + 9]
						* vector.y + rhsMat[rhsMatOffset + 10] * vector.z);
	}

	public Vector3f multiplyM4V(float[] lhsMat, int lhsMatOffset,
			Vector3f vector) {
		return set(vector.x * lhsMat[lhsMatOffset] + vector.y
				* lhsMat[lhsMatOffset + 4] + vector.z
				* lhsMat[lhsMatOffset + 8], vector.x * lhsMat[lhsMatOffset + 1]
				+ vector.y * lhsMat[lhsMatOffset + 5] + vector.z
				* lhsMat[lhsMatOffset + 9], vector.x * lhsMat[lhsMatOffset + 2]
				+ vector.y * lhsMat[lhsMatOffset + 6] + vector.z
				* lhsMat[lhsMatOffset + 10]);
	}

	public Vector3f multiply(final float pX, final float pY, final float pZ) {
		return set(x * pX, y * pY, z * pZ);
	}

	public Vector3f multiply(final Vector3f vector) {
		return multiply(vector.x, vector.y, vector.z);
	}

	public Vector3f scale(final float scalar) {
		return set(x * scalar, y * scalar, z * scalar);
	}

	public Vector3f setScale(final Vector3f vector, final float scalar) {
		return set(vector.x * scalar, vector.y * scalar, vector.z * scalar);
	}

	public float dot(final float pX, final float pY, final float pZ) {
		return x * pX + y * pY + z * pZ;
	}

	public float dot(final Vector3f vector) {
		return dot(vector.x, vector.y, vector.z);
	}

	public Vector3f setCross(final float pX1, final float pY1, final float pZ1,
			final float pX2, final float pY2, final float pZ2) {
		return set(pY1 * pZ2 - pZ1 * pY2, pZ1 * pX2 - pX1 * pZ2, pX1 * pY2
				- pY1 * pX2);
	}

	/**
	 * (AxB)xA = V <br />
	 * V perpendicular to A <br />
	 * V dot B >= 0<br />
	 * <br />
	 * (AxB)xB = V <br />
	 * V perpendicular to B <br />
	 * V dot A <= 0<br />
	 * <br />
	 * Ax(BxA) = (AxB)xA
	 */
	public Vector3f setCross(final Vector3f vector1, final Vector3f vector2) {
		return setCross(vector1.x, vector1.y, vector1.z, vector2.x, vector2.y,
				vector2.z);
	}

	public float crossSqr(final float pX1, final float pY1, final float pZ1,
			final float pX2, final float pY2, final float pZ2) {
		final float x = pY1 * pZ2 - pZ1 * pY2;
		final float y = pZ1 * pX2 - pX1 * pZ2;
		final float z = pX1 * pY2 - pY1 * pX2;
		return x * x + y * y + z * z;
	}

	public float crossSqr(Vector3f vector1, Vector3f vector2) {
		return crossSqr(vector1.x, vector1.y, vector1.z, vector2.x, vector2.y,
				vector2.z);
	}

	public Vector3f setDoubleCross(final float pX1, final float pY1,
			final float pZ1, final float pX2, final float pY2, final float pZ2) {
		final float dot = pX1 * pX1 + pY1 * pY1 + pZ1 * pZ1;
		return set(pX2 * dot, pY2 * dot, pZ2 * dot);
	}

	public Vector3f setCross(final float pX1, final float pY1, final float pZ1,
			final float pX2, final float pY2, final float pZ2, final float pX3,
			final float pY3, final float pZ3) {
		final float x1 = pY1 * pZ2 - pZ1 * pY2;
		final float y1 = pZ1 * pX2 - pX1 * pZ2;
		final float z1 = pX1 * pY2 - pY1 * pX2;
		return set(y1 * pZ3 - z1 * pY3, z1 * pX3 - x1 * pZ3, x1 * pY3 - y1
				* pX3);
	}

	public Vector3f setCross(final Vector3f vector1, final Vector3f vector2,
			final float v1dotv2) {
		final float ca = vector1.dot(vector1);
		return set(vector2.x * ca - vector1.x * v1dotv2, vector2.y * ca
				- vector1.y * v1dotv2, vector2.z * ca - vector1.z * v1dotv2);
	}

	public float norm1() {
		return Math.abs(x) + Math.abs(y) + Math.abs(z);
	}

	public Vector3f norm() {
		final float length2 = dot(this);
		if (length2 != 0 && Math.abs(length2 - 1) > Tolerance.NULL) {
			scale(1F / (float) Math.sqrt(length2));
		}
		return this;
	}

	public float normLength() {
		float length2 = dot(this);
		if (length2 <= Tolerance.NULL)
			return 0;
		if (Math.abs(length2 - 1) <= Tolerance.NULL)
			return 1;
		length2 = (float) Math.sqrt(length2);
		scale(1F / length2);
		return length2;
	}

	public Vector3f setNorm(Vector3f vector) {
		final float length2 = vector.dot(vector);
		if (length2 != 0 && Math.abs(length2 - 1) > Tolerance.NULL) {
			return setScale(vector, 1F / (float) Math.sqrt(length2));
		}
		return set(vector);
	}

	public Vector3f norm(float l) {
		final float length2 = dot(this);
		if (Math.abs(length2) > Tolerance.NULL) {
			if (Math.abs(length2 - 1) > Tolerance.NULL) {
				scale(l / (float) Math.sqrt(length2));
			} else {
				scale(l);
			}
		}
		return this;
	}

	public Vector3f setNorm(Vector3f vector, float l) {
		final float length2 = vector.dot(vector);
		if (length2 != 0 && Math.abs(length2) > Tolerance.NULL) {
			if (Math.abs(length2 - 1) > Tolerance.NULL) {
				return setScale(vector, l / (float) Math.sqrt(length2));
			} else {
				return setScale(vector, l);
			}
		}
		return set(vector);
	}

	public float length() {
		final float length2 = dot(this);
		if (Math.abs(length2) > Tolerance.NULL
				&& Math.abs(length2 - 1) > Tolerance.NULL) {
			return (float) Math.sqrt(length2);
		}
		return length2;
	}

	public static float length(final float x, final float y, final float z) {
		final float length2 = x * x + y * y + z * z;
		if (Math.abs(length2) > Tolerance.NULL
				&& Math.abs(length2 - 1) > Tolerance.NULL) {
			return (float) Math.sqrt(length2);
		}
		return length2;
	}

	public static void lerp(final Vector3f from, final Vector3f to,
			final float alpha) {
		to.x += alpha * (from.x - to.x);
		to.y += alpha * (from.y - to.y);
		to.z += alpha * (from.z - to.z);
	}

	public boolean isZero() {
		return x == 0 && y == 0 && z == 0;
	}

	public boolean isAlmostZero() {
		return Math.abs(x) <= Tolerance.NULL && Math.abs(y) <= Tolerance.NULL
				&& Math.abs(z) <= Tolerance.NULL;
	}

	@Override
	public boolean equals(Object o) {
		if (o == this)
			return true;
		if (o != null && o instanceof Vector3f) {
			final Vector3f v = (Vector3f) o;
			return x == v.x && y == v.y && z == v.z;
		}
		return false;
	}

	@Override
	public String toString() {
		return x + "," + y + "," + z;
	}
}
