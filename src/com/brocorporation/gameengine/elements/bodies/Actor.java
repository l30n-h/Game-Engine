package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.Vector3f;

public class Actor extends RigidBody {

	protected final static byte slowWalking_2 = 1;
	protected final static byte fastWalking_2 = 4;
	protected final static byte running_2 = 16;
	protected final static byte sprint_2 = 100;
	protected final static float slowWalkingAccerleration = 0.5f;
	protected final static float fastWalkingAccerleration = 1;
	protected final static float runningAccerleration = 2;
	protected final static float sprintAccerleration = 5;

	protected float jumpingVelocity = 0;
	protected boolean jump = false;

	public Actor(final IShape shape, final float pMass) {
		super(shape, pMass);
	}

	final static Vector3f f = new Vector3f(0, 0, 0);
	byte walkDirection = 0;
	byte rotateDirection = 0;

	public void setJumpingVelocity(final float vY) {
		jumpingVelocity = vY;
	}

	public void setJumpingHeight(final float height) {
		jumpingVelocity = (float) Math.sqrt(2 * World.GRAVITY * height);
	}

	public void jump() {
		jump = true;
	}

	@Override
	public void prepareUpdatePosition(final IUpdateInfo uInfo) {
		if (isOnGround()) {
			if (walkDirection != 0) {
				f.set(0, 0, walkDirection * sprintAccerleration * mass * uInfo.getRate());
				affineTransform.getOrientation().rotateV(f);
				if (Math.signum(f.getMaxValue()) != Math.signum(linearVelocity.getMaxValue())
						|| linearVelocity.dot() < sprint_2) {
					addImpulse(f.x, f.y, f.z);
				}
			}
			if (jump) {
				linearVelocity.y = jumpingVelocity;
			}

		}
		walkDirection = 0;
		jump = false;
		if (rotateDirection != 0
				&& (rotateDirection != Math.signum(angularVelocity.y) || angularVelocity.dot() < 0.625)) {
			addAngularMomentum(0, rotateDirection * mass * 0.25f * uInfo.getRate(), 0);
			rotateDirection = 0;
		}
		super.prepareUpdatePosition(uInfo);
	}

	public void push(IUpdateInfo uInfo) {
		walkDirection = 1;
	}

	public void pushInverse(IUpdateInfo uInfo) {
		walkDirection = -1;
	}

	public void rotate(IUpdateInfo uInfo) {
		rotateDirection = 1;
	}

	public void rotateInverse(IUpdateInfo uInfo) {
		rotateDirection = -1;
	}
}
