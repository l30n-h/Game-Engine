package com.brocorporation.gameengine.utils;

public class Quaternion extends Vector4f {

	public final static float PIOVER180 = (float) Math.PI / 180;
	public final static float PIOVER360 = (float) Math.PI / 360;

	public Quaternion() {
		super(0, 0, 0, 1);
	}

	public Quaternion set(final float pX, final float pY, final float pZ,
			final float pW) {
		x = pX;
		y = pY;
		z = pZ;
		w = pW;
		return this;
	}

	public Quaternion set(final Quaternion quaternion) {
		x = quaternion.x;
		y = quaternion.y;
		z = quaternion.z;
		w = quaternion.w;
		return this;
	}

	public Quaternion konjugate() {
		return set(-x, -y, -z, w);
	}

	public Quaternion setKonjugate(final Quaternion quaternion) {
		return set(-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w);
	}

	public Quaternion multiply(final float pX, final float pY, final float pZ,
			final float pW) {
		// return set(w*pX + x*pW + y*pZ - z*pY,
		// w*pY + y*pW + z*pX - x*pZ,
		// w*pZ + z*pW + x*pY - y*pX,
		// w*pW - x*pX - y*pY - z*pZ);
		final float t5 = (z + x) * (pX + pY);
		final float t6 = (w + y) * (pW - pZ);
		final float t7 = (w - y) * (pW + pZ);
		final float t8 = t5 + t6 + t7;
		final float t9 = ((z - x) * (pX - pY) + t8) / 2;
		return set((w + x) * (pW + pX) + t9 - t8,
				(w - x) * (pY + pZ) + t9 - t7, (z + y) * (pW - pX) + t9 - t6,
				(z - y) * (pY - pZ) + t9 - t5);
	}

	public Quaternion multiply(final Quaternion quaternion) {
		return multiply(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
	}

	public Quaternion setMultiply(final float pX1, final float pY1,
			final float pZ1, final float pW1, final float pX2, final float pY2,
			final float pZ2, final float pW2) {
		// return set(pW1*pX2 + pX1*pW2 + pY1*pZ2 - pZ1*pY2,
		// pW1*pY2 + pY1*pW2 + pZ1*pX2 - pX1*pZ2,
		// pW1*pZ2 + pZ1*pW2 + pX1*pY2 - pY1*pX2,
		// pW1*pW2 - pX1*pX2 - pY1*pY2 - pZ1*pZ2);
		final float t5 = (pZ1 + pX1) * (pX2 + pY2);
		final float t6 = (pW1 + pY1) * (pW2 - pZ2);
		final float t7 = (pW1 - pY1) * (pW2 + pZ2);
		final float t8 = t5 + t6 + t7;
		final float t9 = ((pZ1 - pX1) * (pX2 - pY2) + t8) / 2;
		return set((pW1 + pX1) * (pW2 + pX2) + t9 - t8, (pW1 - pX1)
				* (pY2 + pZ2) + t9 - t7, (pZ1 + pY1) * (pW2 - pX2) + t9 - t6,
				(pZ1 - pY1) * (pY2 - pZ2) + t9 - t5);
	}

	public Quaternion setMultiply(final Quaternion q1, final Quaternion q2) {
		return setMultiply(q1.x, q1.y, q1.z, q1.w, q2.x, q2.y, q2.z, q2.w);
	}

	public Quaternion integrateRotationEuler(final float pX, final float pY,
			final float pZ) {
		return integrateRotationScaled(pX, pY, pZ, PIOVER180);
	}

	public Quaternion integrateRotationEuler(final Vector3f vector) {
		return integrateRotationEuler(vector.x, vector.y, vector.z);
	}

	public Quaternion integrateRotationScaledEuler(final float pX,
			final float pY, final float pZ, final float scalar) {
		return integrateRotationScaled(pX, pY, pZ, scalar * PIOVER180);
	}

	public Quaternion integrateRotationScaledEuler(final Vector3f vector,
			final float scalar) {
		return integrateRotationScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Quaternion integrateRotation(final Vector3f vector) {
		return integrateRotation(vector.x, vector.y, vector.z);
	}

	public Quaternion integrateRotationScaled(final Vector3f vector,
			final float scalar) {
		return integrateRotationScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Quaternion integrateRotationScaled(final float wx, final float wy,
			final float wz, final float scalar) {
		return integrateRotation(wx * scalar, wy * scalar, wz * scalar);
	}

	public Quaternion addRotationEuler(final Vector3f vector) {
		return addRotationEuler(vector.x, vector.y, vector.z);
	}

	public Quaternion addRotationEuler(final float wx, final float wy,
			final float wz) {
		return addRotation(wx * PIOVER180, wy * PIOVER180, wz * PIOVER180);
	}

	public Quaternion addRotation(final Vector3f vector) {
		return addRotation(vector.x, vector.y, vector.z);
	}

	public Quaternion addRotationScaled(final Vector3f vector,
			final float scalar) {
		return addRotationScaled(vector.x, vector.y, vector.z, scalar);
	}

	public Quaternion addRotationScaled(final float wx, final float wy,
			final float wz, final float scalar) {
		return addRotation(wx * scalar, wy * scalar, wz * scalar);
	}

	public Quaternion addRotation(final float wx, final float wy, final float wz) {
		final float hwx = wx / 2;
		final float hwy = wy / 2;
		final float hwz = wz / 2;
		final float sinxh = (float) Math.sin(hwx);
		final float cosxh = (float) Math.cos(hwx);
		final float sinyh = (float) Math.sin(hwy);
		final float cosyh = (float) Math.cos(hwy);
		final float sinzh = (float) Math.sin(hwz);
		final float coszh = (float) Math.cos(hwz);
		final float x1 = cosxh * x + sinxh * w;
		final float y1 = cosxh * y - sinxh * z;
		final float z1 = cosxh * z + sinxh * y;
		final float w1 = cosxh * w - sinxh * x;
		final float x2 = cosyh * x1 + sinyh * z1;
		final float y2 = cosyh * y1 + sinyh * w1;
		final float z2 = cosyh * z1 - sinyh * x1;
		final float w2 = cosyh * w1 - sinyh * y1;

		set(coszh * x2 - sinzh * y2, coszh * y2 + sinzh * x2, coszh * z2
				+ sinzh * w2, coszh * w2 - sinzh * z2);
		return this;
	}

	static Quaternion tmp = new Quaternion();

	public Quaternion integrateRotation(final float wx, final float wy,
			final float wz) {
		tmp.setMultiply(wx / 2, wy / 2, wz / 2, 0, x, y, z, w);
		add(tmp);
		norm();

		// final float sq = wx * wx + wy * wy + wz * wz;
		// float qw;
		// float s;
		// if (sq < 0.001691455f) {// TODO sq*sq/27 < eps 60fps vs free
		// qw = 1 - sq * 0.125f;
		// s = 0.5f - sq * 0.02083333333f;
		// } else {
		// final float thetaMag = (float) Math.sqrt(sq);
		// final float halfthetaMag = thetaMag /2;
		// qw = (float) Math.cos(halfthetaMag);
		// s = (float) Math.sin(halfthetaMag) / thetaMag;
		// }
		// setMultiply(wx * s, wy * s, wz * s, qw, x, y, z, w);
		return this;
	}

	public Vector3f rotateV(final Vector3f vector) {
		return rotateV(vector, vector);
	}

	public Vector3f rotateV(final Vector3f result, final Vector3f vector) {
		final float tx = y * vector.z - z * vector.y;
		final float ty = z * vector.x - x * vector.z;
		final float tz = x * vector.y - y * vector.x;
		return result.setAdd(vector.x, vector.y, vector.z, (w * tx + y * tz - z
				* ty) * 2, (w * ty + z * tx - x * tz) * 2, (w * tz + x * ty - y
				* tx) * 2);
	}

	public Vector3f rotateInverseV(final Vector3f result, final Vector3f vector) {
		final float tx = y * vector.z - z * vector.y;
		final float ty = z * vector.x - x * vector.z;
		final float tz = x * vector.y - y * vector.x;
		return result.setAdd(vector.x, vector.y, vector.z,
				(-w * tx + y * tz - z * ty) * 2,
				(-w * ty + z * tx - x * tz) * 2,
				(-w * tz + x * ty - y * tx) * 2);
	}

	public Vector3f rotateHalfsize(final Vector3f result, final Vector3f vector) {
		final float vx2 = vector.x * 2;
		final float vy2 = vector.y * 2;
		final float vz2 = vector.z * 2;
		final float t1y = z * vx2;
		final float t1z = y * vx2;
		final float t2x = z * vy2;
		final float t2z = x * vy2;
		final float t3x = y * vz2;
		final float t3y = x * vz2;
		final float rX = Math.abs(z * t1y + y * t1z - vector.x)
				+ Math.abs(y * t2z - w * t2x) + Math.abs(w * t3x + z * t3y);
		final float rY = Math.abs(w * t1y + x * t1z)
				+ Math.abs(x * t2z + z * t2x - vector.y)
				+ Math.abs(z * t3x - w * t3y);
		final float rZ = Math.abs(x * t1y - w * t1z)
				+ Math.abs(w * t2z + y * t2x)
				+ Math.abs(y * t3x + x * t3y - vector.z);
		return result.set(rX, rY, rZ);
	}

	public Vector3f rotateHalfsize(final Vector3f vector) {
		return rotateHalfsize(vector, vector);
	}

	public Quaternion getQuaternion(final Vector3f vector1,
			final Vector3f vector2) {
		w = (float) Math.sqrt(vector1.dot(vector1) * vector2.dot(vector2))
				+ vector1.dot(vector2);
		((Quaternion) setCross(vector1, vector2)).norm();
		return this;
	}

	public Quaternion getQuaternion(final Vector3f vector, float rad) {
		rad /= 2;
		set(vector).norm().scale((float) Math.sin(rad));
		w = (float) Math.cos(rad);
		return this;
	}

	public Quaternion getQuaternionEuler(final float degX, final float degY,
			final float degZ) {
		return getQuaternion(degX * PIOVER360, degY * PIOVER360, degZ
				* PIOVER360);
	}

	public Quaternion getQuaternionEuler(Vector3f rotation) {
		return getQuaternion(rotation.x * PIOVER360, rotation.y * PIOVER360,
				rotation.z * PIOVER360);
	}

	public Quaternion getQuaternion(Vector3f rotation) {
		return getQuaternion(rotation.x, rotation.y, rotation.z);
	}

	public Quaternion getQuaternion(final float x, final float y, final float z) {
		final float siny = (float) Math.sin(y);
		final float sinz = (float) Math.sin(z);
		final float sinx = (float) Math.sin(x);
		final float cosy = (float) Math.cos(y);
		final float cosz = (float) Math.cos(z);
		final float cosx = (float) Math.cos(x);
		final float sinxcosy = sinx * cosy;
		final float sinxsiny = sinx * siny;
		final float cosxcosy = cosx * cosy;
		final float cosxsiny = cosx * siny;
		set(sinxcosy * cosz - cosxsiny * sinz,
				cosxsiny * cosz + sinxcosy * sinz,
				cosxcosy * sinz - sinxsiny * cosz,
				cosxcosy * cosz + sinxsiny * sinz).norm();
		return this;
	}

	public void multiplyMQ(float[] result, int resultOffset, float[] lhs,
			int rhsOffset) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;

		final float r0 = 1 - 2 * (y2 + z2);
		final float r1 = 2 * (xy - wz);
		final float r2 = 2 * (xz + wy);
		final float r4 = 2 * (xy + wz);
		final float r5 = 1 - 2 * (x2 + z2);
		final float r6 = 2 * (yz - wx);
		final float r8 = 2 * (xz - wy);
		final float r9 = 2 * (yz + wx);
		final float r10 = 1 - 2 * (x2 + y2);

		final float l0 = lhs[rhsOffset];
		final float l1 = lhs[rhsOffset + 1];
		final float l2 = lhs[rhsOffset + 2];
		final float l4 = lhs[rhsOffset + 4];
		final float l5 = lhs[rhsOffset + 5];
		final float l6 = lhs[rhsOffset + 6];
		final float l8 = lhs[rhsOffset + 8];
		final float l9 = lhs[rhsOffset + 9];
		final float l10 = lhs[rhsOffset + 10];
		final float l12 = lhs[rhsOffset + 12];
		final float l13 = lhs[rhsOffset + 13];
		final float l14 = lhs[rhsOffset + 14];
		result[resultOffset] = l0 * r0 + l1 * r4 + l2 * r8;
		result[resultOffset + 1] = l0 * r1 + l1 * r5 + l2 * r9;
		result[resultOffset + 2] = l0 * r2 + l1 * r6 + l2 * r10;
		result[resultOffset + 3] = lhs[rhsOffset + 3];
		result[resultOffset + 4] = l4 * r0 + l5 * r4 + l6 * r8;
		result[resultOffset + 5] = l4 * r1 + l5 * r5 + l6 * r9;
		result[resultOffset + 6] = l4 * r2 + l5 * r6 + l6 * r10;
		result[resultOffset + 7] = lhs[rhsOffset + 7];
		result[resultOffset + 8] = l8 * r0 + l9 * r4 + l10 * r8;
		result[resultOffset + 9] = l8 * r1 + l9 * r5 + l10 * r9;
		result[resultOffset + 10] = l8 * r2 + l9 * r6 + l10 * r10;
		result[resultOffset + 11] = lhs[rhsOffset + 11];
		result[resultOffset + 12] = l12 * r0 + l13 * r4 + l14 * r8;
		result[resultOffset + 13] = l12 * r1 + l13 * r5 + l14 * r9;
		result[resultOffset + 14] = l12 * r2 + l13 * r6 + l14 * r10;
		result[resultOffset + 15] = lhs[rhsOffset + 15];
	}

	public void multiplyQM(float[] result, int resultOffset, float[] rhs,
			int lhsOffset) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;

		final float l0 = 1 - 2 * (y2 + z2);
		final float l1 = 2 * (xy - wz);
		final float l2 = 2 * (xz + wy);
		final float l4 = 2 * (xy + wz);
		final float l5 = 1 - 2 * (x2 + z2);
		final float l6 = 2 * (yz - wx);
		final float l8 = 2 * (xz - wy);
		final float l9 = 2 * (yz + wx);
		final float l10 = 1 - 2 * (x2 + y2);

		final float r0 = rhs[lhsOffset];
		final float r1 = rhs[lhsOffset + 1];
		final float r2 = rhs[lhsOffset + 2];
		final float r3 = rhs[lhsOffset + 3];
		final float r4 = rhs[lhsOffset + 4];
		final float r5 = rhs[lhsOffset + 5];
		final float r6 = rhs[lhsOffset + 6];
		final float r7 = rhs[lhsOffset + 7];
		final float r8 = rhs[lhsOffset + 8];
		final float r9 = rhs[lhsOffset + 9];
		final float r10 = rhs[lhsOffset + 10];
		final float r11 = rhs[lhsOffset + 11];
		result[resultOffset] = l0 * r0 + l1 * r4 + l2 * r8;
		result[resultOffset + 1] = l0 * r1 + l1 * r5 + l2 * r9;
		result[resultOffset + 2] = l0 * r2 + l1 * r6 + l2 * r10;
		result[resultOffset + 3] = l0 * r3 + l1 * r7 + l2 * r11;
		result[resultOffset + 4] = l4 * r0 + l5 * r4 + l6 * r8;
		result[resultOffset + 5] = l4 * r1 + l5 * r5 + l6 * r9;
		result[resultOffset + 6] = l4 * r2 + l5 * r6 + l6 * r10;
		result[resultOffset + 7] = l4 * r3 + l5 * r7 + l6 * r11;
		result[resultOffset + 8] = l8 * r0 + l9 * r4 + l10 * r8;
		result[resultOffset + 9] = l8 * r1 + l9 * r5 + l10 * r9;
		result[resultOffset + 10] = l8 * r2 + l9 * r6 + l10 * r10;
		result[resultOffset + 11] = l8 * r3 + l9 * r7 + l10 * r11;
		result[resultOffset + 12] = rhs[lhsOffset + 12];
		result[resultOffset + 13] = rhs[lhsOffset + 13];
		result[resultOffset + 14] = rhs[lhsOffset + 14];
		result[resultOffset + 15] = rhs[lhsOffset + 15];
	}

	public float[] getRotationMatrix(final float[] result) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;
		result[0] = 1 - 2 * (y2 + z2);
		result[1] = 2 * (xy - wz);
		result[2] = 2 * (xz + wy);
		result[3] = 0;
		result[4] = 2 * (xy + wz);
		result[5] = 1 - (x2 + z2);
		result[6] = 2 * (yz - wx);
		result[7] = 0;
		result[8] = 2 * (xz - wy);
		result[9] = 2 * (yz + wx);
		result[10] = 1 - 2 * (x2 + y2);
		result[11] = 0;
		result[12] = 0;
		result[13] = 0;
		result[14] = 0;
		result[15] = 1;
		return result;
	}

	public float[] getRotationMatrix3(final float[] result) {
		final float x2 = x * x;
		final float y2 = y * y;
		final float z2 = z * z;
		final float xy = x * y;
		final float xz = x * z;
		final float yz = y * z;
		final float wx = w * x;
		final float wy = w * y;
		final float wz = w * z;
		result[0] = 1 - 2 * (y2 + z2);
		result[1] = 2 * (xy - wz);
		result[2] = 2 * (xz + wy);
		result[3] = 2 * (xy + wz);
		result[4] = 1 - 2 * (x2 + z2);
		result[5] = 2 * (yz - wx);
		result[6] = 2 * (xz - wy);
		result[7] = 2 * (yz + wx);
		result[8] = 1 - 2 * (x2 + y2);
		return result;
	}

	public Quaternion lookAt(final float pFromX, final float pFromY,
			final float pFromZ, final float pToX, final float pToY,
			final float pToZ) {
		setSubtract(pToX, pToY, pToZ, pFromX, pFromY, pFromZ);
		return lookAt(x, y, z, length());
	}

	public Quaternion lookAt(final float[] quaternion, final float pDirX,
			final float pDirY, final float pDirZ) {
		set(pDirX, pDirY, pDirZ);
		return lookAt(pDirX, pDirY, pDirZ, length());
	}

	public Quaternion lookAt(final float pDirX, final float pDirY,
			final float pDirZ, final float length) {
		set(-pDirY, pDirX, 0, length + pDirZ).norm();
		return this;
	}

	public static void getHalfRadians(final Vector3f vector, final float degX,
			final float degY, final float degZ) {
		vector.set(degX * PIOVER360, degY * PIOVER360, degZ * PIOVER360);
	}
}
