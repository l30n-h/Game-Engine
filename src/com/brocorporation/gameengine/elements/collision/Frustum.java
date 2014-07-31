package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class Frustum {

	public final static byte NONE = 0;
	public final static byte CONTAINS = 1;
	public final static byte INTERSECTS = 2;

	protected final static byte FRONT = 0;
	protected final static byte BACK = 1;
	protected final static byte RIGHT = 2;
	protected final static byte LEFT = 3;
	protected final static byte BOTTOM = 4;
	protected final static byte TOP = 5;

	protected final Vector4f[] planes = { new Vector4f(), new Vector4f(),
			new Vector4f(), new Vector4f(), new Vector4f(), new Vector4f() };
	protected float m12, m13, m14, m15;
	protected boolean hasChanged;
	protected final static Vector3f temp = new Vector3f();
	protected float[] lastFrustum = new float[16];

	public Frustum() {
	}

	public void setFrustum(final float[] frustumM) {
		hasChanged = m12 != frustumM[12] || m13 != frustumM[13]
				|| m14 != frustumM[14] || m15 != frustumM[15];
		if (hasChanged) {
			m12 = frustumM[12];
			m13 = frustumM[13];
			m14 = frustumM[14];
			m15 = frustumM[15];
			planes[FRONT].x = frustumM[3] + frustumM[2];
			planes[FRONT].y = frustumM[7] + frustumM[6];
			planes[FRONT].z = frustumM[11] + frustumM[10];
			planes[FRONT].w = -frustumM[15] - frustumM[14];

			planes[BACK].x = frustumM[3] - frustumM[2];
			planes[BACK].y = frustumM[7] - frustumM[6];
			planes[BACK].z = frustumM[11] - frustumM[10];
			planes[BACK].w = frustumM[14] - frustumM[15];

			planes[RIGHT].x = frustumM[3] - frustumM[0];
			planes[RIGHT].y = frustumM[7] - frustumM[4];
			planes[RIGHT].z = frustumM[11] - frustumM[8];
			planes[RIGHT].w = frustumM[12] - frustumM[15];

			planes[LEFT].x = frustumM[3] + frustumM[0];
			planes[LEFT].y = frustumM[7] + frustumM[4];
			planes[LEFT].z = frustumM[11] + frustumM[8];
			planes[LEFT].w = -frustumM[15] - frustumM[12];

			planes[TOP].x = frustumM[3] - frustumM[1];
			planes[TOP].y = frustumM[7] - frustumM[5];
			planes[TOP].z = frustumM[11] - frustumM[9];
			planes[TOP].w = frustumM[13] - frustumM[15];

			planes[BOTTOM].x = frustumM[3] + frustumM[1];
			planes[BOTTOM].y = frustumM[7] + frustumM[5];
			planes[BOTTOM].z = frustumM[11] + frustumM[9];
			planes[BOTTOM].w = -frustumM[15] - frustumM[13];

			normPlane(planes[FRONT]);
			normPlane(planes[BACK]);
			normPlane(planes[RIGHT]);
			normPlane(planes[LEFT]);
			normPlane(planes[TOP]);
			normPlane(planes[BOTTOM]);
		}
	}

	public boolean hasChanged() {
		return hasChanged;
	}

	private static void normPlane(Vector4f plane) {
		final Vector3f v = plane;
		final float length2 = v.dot(v);
		plane.scale(1F / (float) Math.sqrt(length2));
	}

	public boolean intersects(final Vector3f vertex) {
		for (final Vector4f p : planes) {
			if (p.x * vertex.x + p.y * vertex.y + p.z * vertex.z < p.w) {
				return false;
			}
		}
		return true;
	}

	public boolean intersects(final Vector3f vertex, final float radius) {
		for (final Vector4f p : planes) {
			if (p.x * vertex.x + p.y * vertex.y + p.z * vertex.z < p.w - radius) {
				return false;
			}
		}
		return true;
	}

	public byte containsIntersects(final IShape shape) {
		if (hasChanged || shape.hasChanged()) {
			Vector4f p;
			for (int i = 0; i < 6; i++) {
				p = planes[i];
				shape.getMinAlongDirection(temp, p);
				if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
					for (; i < 6; i++) {
						p = planes[i];
						shape.getMaxAlongDirection(temp, p);
						if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
							shape.setFrustumType(NONE);
							return NONE;
						}
					}
					shape.setFrustumType(INTERSECTS);
					return INTERSECTS;
				}
			}
			shape.setFrustumType(CONTAINS);
			return CONTAINS;
		}
		return shape.getFrustumType();
	}

	public boolean intersects(final IShape shape) {
		if (hasChanged || shape.hasChanged()) {
			for (final Vector4f p : planes) {
				shape.getMaxAlongDirection(temp, p);
				if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
					shape.setFrustumType(NONE);
					return false;
				}
			}
			shape.setFrustumType(INTERSECTS);
			return true;
		}
		return shape.getFrustumType() != NONE;
	}

	public boolean contains(final IShape shape) {
		if (hasChanged || shape.hasChanged()) {
			for (final Vector4f p : planes) {
				shape.getMinAlongDirection(temp, p);
				if (p.x * temp.x + p.y * temp.y + p.z * temp.z < p.w) {
					shape.setFrustumType(NONE);
					return false;
				}
			}
			shape.setFrustumType(CONTAINS);
			return true;
		}
		return shape.getFrustumType() == CONTAINS;
	}
}