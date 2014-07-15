package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.Vector3f;

public class DynamicBody extends StaticBody {

	public final static byte INFINITY_MASS = 0;
	protected final static float MIN_VELOCITY2 = 0.01F;

	protected final float mass;
	protected final float inverseMass;

	protected final Vector3f positionCorrection = new Vector3f();

	protected final Vector3f linearVelocity = new Vector3f();
	protected final Vector3f linearMomentum = new Vector3f();

	protected AABB sweptAABB = new AABB();// TODO temporary

	protected float maxVelocity;
	protected float maxVelocity2 = Float.POSITIVE_INFINITY;

	protected boolean gravityEnabled, isOnGround, updateLinearMomentum,
			isLinearMoving;

	public DynamicBody(final IShape shape, final float pMass) {
		super(shape);
		mass = pMass;
		if (mass != INFINITY_MASS) {
			inverseMass = 1F / mass;
		} else {
			inverseMass = INFINITY_MASS;
		}
	}

	public float getMass() {
		return mass;
	}

	public float getInverseMass() {
		return inverseMass;
	}

	public void isOnGround(final boolean pIsOnGround) {
		isOnGround = pIsOnGround;
	}

	public boolean isOnGround() {
		return isOnGround;
	}

	public Vector3f getPositionCorrection() {
		return positionCorrection;
	}

	public Vector3f getLinearVelocity() {
		return linearVelocity;
	}

	public void isGravityEnabled(final boolean pGravityEnabled) {
		gravityEnabled = pGravityEnabled && mass != INFINITY_MASS;
	}

	public void setMaxVelocity(final float max) {
		if (max >= 0) {
			maxVelocity = max;
			maxVelocity2 = max * max;
		} else {
			maxVelocity2 = Float.POSITIVE_INFINITY;
		}
	}

	public void addImpulse(final float impulseX, final float impulseY,
			final float impulseZ) {
		if (impulseX != 0 || impulseY != 0 || impulseZ != 0) {
			linearMomentum.add(impulseX, impulseY, impulseZ);
			updateLinearMomentum = true;
		}
	}

	public void clearMomenta() {
		linearMomentum.set(0, 0, 0);
		updateLinearMomentum = false;
	}

	protected void applyMomenta() {
		if (updateLinearMomentum) {
			linearVelocity.addScaled(linearMomentum, inverseMass);
			clearMomenta();
		}
	}

	@Override
	public void prepareUpdatePosition(final IUpdateInfo uInfo) {
		applyMomenta();
		if (gravityEnabled) {
			linearVelocity.y -= World.GRAVITY * uInfo.getRate();
		}

		final float vv = linearVelocity.dot(linearVelocity);
		if (isLinearMoving = (vv > MIN_VELOCITY2)) {
			isOnGround(false);
			if (vv > maxVelocity2) {
				linearVelocity.scale(maxVelocity / (float) Math.sqrt(vv));
			}
		}
	}

	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		super.updatePosition(uInfo);
		final Vector3f translation = affineTransform.getTranslation();
		if (isLinearMoving
				&& linearVelocity.dot(linearVelocity) > MIN_VELOCITY2) {
			translation.addScaled(linearVelocity, uInfo.getRate());
			updateTranslation = true;
		}
		if (!positionCorrection.isZero()) {
			translation.subtract(positionCorrection);
			updateTranslation = true;
			positionCorrection.set(0, 0, 0);
		}
	}

	public void generateSweptBounds(final IUpdateInfo uInfo) {
		if (isLinearMoving) {
			final float halfdTime = uInfo.getHalfRate();
			final float vX = linearVelocity.x * halfdTime;
			final float vY = linearVelocity.y * halfdTime;
			final float vZ = linearVelocity.z * halfdTime;
			final Vector3f hs = shape.getAABB().getHalfsize();
			final Vector3f p = getPosition();
			sweptAABB.setPosition(p.x + vX, p.y + vY, p.z + vZ);
			sweptAABB.setHalfsize(hs.x + Math.abs(vX), hs.y + Math.abs(vY),
					hs.z + Math.abs(vZ));
		} else {
			final AABB aabb = shape.getAABB();
			sweptAABB.setPosition(aabb.getPosition());
			final Vector3f h = aabb.getHalfsize();
			sweptAABB.setHalfsize(h.x, h.y, h.z);
		}
	}

	public boolean isLinearMoving() {
		return isLinearMoving;
	}

	public AABB getSweptAABB() {
		return sweptAABB;
	}
}
