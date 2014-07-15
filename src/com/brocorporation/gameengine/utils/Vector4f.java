package com.brocorporation.gameengine.utils;

public class Vector4f extends Vector3f {

	public float w;

	public Vector4f() {
		this(0, 0, 0, 0);
	}

	public Vector4f(final float pX, final float pY, final float pZ,
			final float pW) {
		set(pX, pY, pZ, pW);
	}

	public Vector4f(final Vector4f vector) {
		this(vector.x, vector.y, vector.z, vector.w);
	}

	public Vector4f set(final float pX, final float pY, final float pZ,
			final float pW) {
		x = pX;
		y = pY;
		z = pZ;
		w = pW;
		return this;
	}

	public Vector4f set(final Vector4f vector) {
		return set(vector.x, vector.y, vector.z, vector.w);
	}

	public Vector4f abs() {
		x = Math.abs(x);
		y = Math.abs(y);
		z = Math.abs(z);
		return this;
	}

	public Vector4f add(final float pX, final float pY, final float pZ,
			final float pW) {
		x += pX;
		y += pY;
		z += pZ;
		w += pW;
		return this;
	}

	public Vector4f add(final Vector4f vector) {
		return add(vector.x, vector.y, vector.z, vector.w);
	}

	public Vector4f addScaled(final float pX, final float pY, final float pZ,
			final float pW, final float scalar) {
		x += pX * scalar;
		y += pY * scalar;
		z += pZ * scalar;
		w += pW * scalar;
		return this;
	}

	public Vector4f addScaled(final Vector4f vector, final float scalar) {
		return addScaled(vector.x, vector.y, vector.z, vector.w, scalar);
	}

	public Vector4f subtract(final float pX, final float pY, final float pZ,
			final float pW) {
		x -= pX;
		y -= pY;
		z -= pZ;
		w -= pW;
		return this;
	}

	public Vector4f subtract(final Vector4f vector) {
		return subtract(vector.x, vector.y, vector.z, vector.w);
	}

	public Vector4f subtractScaled(final float pX, final float pY,
			final float pZ, final float pW, final float scalar) {
		x -= pX * scalar;
		y -= pY * scalar;
		z -= pZ * scalar;
		w -= pW * scalar;
		return this;
	}

	public Vector4f subtractScaled(final Vector4f vector, final float scalar) {
		return subtractScaled(vector.x, vector.y, vector.z, vector.w, scalar);
	}

	public Vector4f multiply(final float pX, final float pY, final float pZ,
			final float pW) {
		x *= pX;
		y *= pY;
		z *= pZ;
		w *= pW;
		return this;
	}

	public Vector4f multiply(final Vector4f vector) {
		return multiply(vector.x, vector.y, vector.z, vector.w);
	}

	public Vector4f scale(final float scalar) {
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
		return this;
	}

	public Vector4f setScale(final Vector4f vector, final float scalar) {
		return set(vector.x * scalar, vector.y * scalar, vector.z * scalar,
				vector.w * scalar);
	}

	public float dot(final float pX, final float pY, final float pZ,
			final float pW) {
		return x * pX + y * pY + z * pZ + w * pW;
	}

	public float dot(final Vector4f vector) {
		return dot(vector.x, vector.y, vector.z, vector.w);
	}

	public float norm1() {
		return Math.abs(x) + Math.abs(y) + Math.abs(z) + Math.abs(w);
	}

	public Vector4f norm() {
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

	public Vector4f setNorm(Vector4f vector) {
		final float length2 = vector.dot(vector);
		if (length2 != 0 && Math.abs(length2 - 1) > Tolerance.NULL) {
			return setScale(vector, 1F / (float) Math.sqrt(length2));
		}
		return set(vector);
	}

	public Vector4f norm(float l) {
		final float length2 = dot(this);
		if (Math.abs(length2) > Tolerance.NULL) {
			if (Math.abs(length2 - 1) > Tolerance.NULL) {
				scale(l);
			} else {
				scale(l / (float) Math.sqrt(length2));
			}
		}
		return this;
	}

	public Vector4f setNorm(Vector4f vector, float l) {
		final float length2 = vector.dot(vector);
		if (length2 != 0 && Math.abs(length2) > Tolerance.NULL) {
			if (Math.abs(length2 - 1) > Tolerance.NULL) {
				return setScale(vector, l);
			} else {
				return setScale(vector, l / (float) Math.sqrt(length2));
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

	public static float length(final float x, final float y, final float z,
			final float w) {
		final float length2 = x * x + y * y + z * z + w * w;
		if (Math.abs(length2) > Tolerance.NULL
				&& Math.abs(length2 - 1) > Tolerance.NULL) {
			return (float) Math.sqrt(length2);
		}
		return length2;
	}

	public boolean isZero() {
		return x == 0 && y == 0 && z == 0 && w == 0;
	}

	public boolean isAlmostZero() {
		return Math.abs(x) <= Tolerance.NULL && Math.abs(y) <= Tolerance.NULL
				&& Math.abs(z) <= Tolerance.NULL
				&& Math.abs(w) <= Tolerance.NULL;
	}

	@Override
	public boolean equals(Object o) {
		if (o == this)
			return true;
		if (o != null && o instanceof Vector4f) {
			final Vector4f v = (Vector4f) o;
			return x == v.x && y == v.y && z == v.z && w == v.w;
		}
		return false;
	}

	@Override
	public String toString() {
		return x + "," + y + "," + z + "," + w;
	}
}
