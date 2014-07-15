package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Vector3f;

public class Sphere extends AABB {

	protected float radius;

	public Sphere(float r) {
		radius = r;
		setHalfsize(r, r, r);
	}

	public float getRadius() {
		return radius;
	}

	@Override
	public void updateBounds(AffineTransform affineTransform,
			boolean updateTranslation, boolean updateOrientation) {
		super.updateBounds(affineTransform, updateTranslation, false);
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

	@Override
	public Vector3f getMaxAlongDirection(Vector3f result, Vector3f dir) {
		return result.setNorm(dir, radius).add(position);
	}

	@Override
	public Vector3f getMinAlongDirection(Vector3f result, Vector3f dir) {
		return result.setNorm(dir, -radius).add(position);
	}
}
