package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.Vector3f;

public class Actor extends RigidBody {

	protected final static byte slowWalking = 1;
	protected final static byte fastWalking = 2;
	protected final static byte running = 4;
	protected final static byte sprint = 10;

	protected float jumpingVelocity = 0;
	protected boolean jump = false;

	public Actor(final IShape shape, final float pMass) {
		super(shape, pMass);
	}

	final static Vector3f f = new Vector3f(0, 0, 0);
	boolean updateWalk = false;
	float walkspeed = 0;

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
			if (updateWalk) {
				f.set(0, 0, walkspeed);
				affineTransform.getOrientation().rotateV(f);
				linearVelocity.set(f);
			} else
				walkspeed = 0;
			if (jump) {
				linearVelocity.y = jumpingVelocity;
			}

		}
		updateWalk = false;
		jump = false;
		super.prepareUpdatePosition(uInfo);
	}

	public void push(IUpdateInfo uInfo) {
		if (walkspeed == 0 || !linearVelocity.isZero()) {
			updateWalk = true;
			walkspeed = Math.min(walkspeed + 2 * uInfo.getRate(), sprint);
		}
	}

	public void pushInverse(IUpdateInfo uInfo) {
		if (walkspeed == 0 || !linearVelocity.isZero()) {
			updateWalk = true;
			walkspeed = Math.max(walkspeed - 2 * uInfo.getRate(), -sprint);
		}
	}
}
