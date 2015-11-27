package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.Vector3f;

public class DynamicBody extends StaticBody {

	protected final static float MIN_VELOCITY2 = -0.001F;// TODO

	protected final float mass;
	protected final float inverseMass;

	protected final Vector3f linearVelocity = new Vector3f();
	protected final Vector3f linearMomentum = new Vector3f();

	protected AABB sweptAABB = new AABB();// TODO temporary

	protected float maxVelocity;
	protected float maxVelocity2 = Float.POSITIVE_INFINITY;

	protected boolean gravityEnabled, isOnGround, updateLinearMomentum, isLinearMoving;

	public DynamicBody(final IShape shape, final float pMass) {
		super(shape);
		mass = pMass;
		if (mass != INFINITY_MASS) {
			inverseMass = 1F / mass;
		} else {
			inverseMass = INFINITY_MASS;
			canActiveCollide(false);
		}
	}

	@Override
	public float getMass() {
		return mass;
	}

	@Override
	public float getInverseMass() {
		return inverseMass;
	}

	public void isOnGround(final boolean pIsOnGround) {
		isOnGround = pIsOnGround;
	}

	public boolean isOnGround() {
		return isOnGround;
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

	public void addImpulse(final float impulseX, final float impulseY, final float impulseZ) {
		if (impulseX != 0 || impulseY != 0 || impulseZ != 0) {
			linearMomentum.add(impulseX, impulseY, impulseZ);
			updateLinearMomentum = true;
		}
	}

	public void clearLinearMomentum() {
		linearMomentum.set(0, 0, 0);
		updateLinearMomentum = false;
	}

	public void clearMomenta() {
		clearLinearMomentum();
	}

	protected void applyMomenta() {
		if (updateLinearMomentum) {
			linearVelocity.addScaled(linearMomentum, inverseMass);
			clearLinearMomentum();
		}
	}

	@Override
	public void prepareUpdatePosition(final IUpdateInfo uInfo) {
		applyMomenta();
		if (gravityEnabled) {
			linearVelocity.y -= World.GRAVITY * uInfo.getRate();
		}

		final float vv = linearVelocity.dot();
		isLinearMoving = (vv > MIN_VELOCITY2);
		if (isLinearMoving) {
			isOnGround(false);
			if (vv > maxVelocity2) {
				linearVelocity.scale(maxVelocity / (float) Math.sqrt(vv));
			}
		}
	}

	// Vector3f oldlinVel = new Vector3f();
	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		if (isLinearMoving && linearVelocity.dot() > MIN_VELOCITY2) {
			// oldlinVel.add(linearVelocity).scale(0.5f);
			// affineTransform.getTranslation().addScaled(oldlinVel,
			// uInfo.getRate());
			// oldlinVel.set(linearVelocity);
			affineTransform.getTranslation().addScaled(linearVelocity, uInfo.getRate());
			updateTranslation = true;
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
			sweptAABB.setHalfsize(hs.x + Math.abs(vX), hs.y + Math.abs(vY), hs.z + Math.abs(vZ));
		} else {
			final AABB aabb = shape.getAABB();
			sweptAABB.setPosition(aabb.getPosition());
			final Vector3f h = aabb.getHalfsize();
			sweptAABB.setHalfsize(h.x, h.y, h.z);
		}
	}

	public void generateSweptBoundsBackwards(final IUpdateInfo uInfo) {
		if (isLinearMoving) {
			final float halfdTime = uInfo.getHalfRate();
			final float vX = linearVelocity.x * halfdTime;
			final float vY = linearVelocity.y * halfdTime;
			final float vZ = linearVelocity.z * halfdTime;
			final Vector3f hs = shape.getAABB().getHalfsize();
			final Vector3f p = getPosition();
			sweptAABB.setPosition(p.x - vX, p.y - vY, p.z - vZ);
			sweptAABB.setHalfsize(hs.x + Math.abs(vX), hs.y + Math.abs(vY), hs.z + Math.abs(vZ));
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
