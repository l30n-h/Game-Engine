package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Vector3f;

public class Sphere implements IShape {

	protected final AABB aabb = new AABB();
	protected float radius;

	public Sphere(float r) {
		radius = r;
		aabb.setHalfsize(r, r, r);
	}

	public float getRadius() {
		return radius;
	}

	public void setPosition(float x, float y, float z) {
		aabb.setPosition(x, y, z);
	}

	public void setPosition(Vector3f v) {
		aabb.setPosition(v);
	}

	@Override
	public void updateBounds(AffineTransform affineTransform,
			boolean updateTranslation, boolean updateOrientation) {
		aabb.updateBounds(affineTransform, updateTranslation, false);
	}

	@Override
	public Vector3f getMaxAlongDirection(Vector3f result, Vector3f dir) {
		return result.setNorm(dir, radius).add(getPosition());
	}

	@Override
	public Vector3f getMinAlongDirection(Vector3f result, Vector3f dir) {
		return result.setNorm(dir, -radius).add(getPosition());
	}

	@Override
	public Vector3f[] getAllMaxAlongDirection(Vector3f[] result, Vector3f dir,
			int count, float eps) {
		if (count > 0) {
			getMaxAlongDirection(result[0], dir);
			for (int j = 1; j < count; j++) {
				result[j].set(0, 0, 0);
			}
		}
		return result;
	}

	@Override
	public Vector3f[] getAllMinAlongDirection(Vector3f[] result, Vector3f dir,
			int count, float eps) {
		if (count > 0) {
			getMinAlongDirection(result[0], dir);
			for (int j = 1; j < count; j++) {
				result[j].set(0, 0, 0);
			}
		}
		return result;
	}

	@Override
	public void getInverseInertiaTensor(Vector3f diagInverseInertiaTensor,
			float inverseMass) {
		diagInverseInertiaTensor.x = diagInverseInertiaTensor.y = diagInverseInertiaTensor.z = inverseMass
				/ (radius * radius * 4);
	}

	@Override
	public Vector3f getPosition() {
		return aabb.getPosition();
	}

	@Override
	public AABB getAABB() {
		return aabb;
	}

	@Override
	public void setFrustumType(byte type) {
		aabb.setFrustumType(type);
	}

	@Override
	public byte getFrustumType() {
		return aabb.getFrustumType();
	}

	@Override
	public boolean hasChanged() {
		return aabb.hasChanged();
	}

	public boolean intersects(final Sphere sphere) {
		final Vector3f position1 = getPosition();
		final Vector3f position2 = sphere.getPosition();
		final float x = position2.x - position1.x;
		final float y = position2.z - position1.y;
		final float z = position2.y - position1.z;
		final float r = sphere.getRadius() + getRadius();
		return x * x + y * y + z * z <= r * r;
	}

	public float getDistance(final Vector3f outNormal, final Sphere sphere) {
		outNormal.setSubtract(sphere.getPosition(), getPosition());
		final float l = outNormal.length();
		outNormal.scale(1F / l);
		return l - (getRadius() + sphere.getRadius());
	}

	public interface IBounds {
		public Sphere getSphere();
	}
}
