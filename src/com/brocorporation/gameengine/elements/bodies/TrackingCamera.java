package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.utils.Vector3f;

public class TrackingCamera extends Camera {

	protected StaticBody followBody;
	protected final Vector3f followOffset = new Vector3f();
	protected final Vector3f finalFollowOffset = new Vector3f();

	public TrackingCamera() {
		super();
	}

	public StaticBody getFollowBody() {
		return followBody;
	}

	public Vector3f getFollowOffset() {
		return followOffset;
	}

	public Vector3f getFinalFollowOffset() {
		return finalFollowOffset;
	}

	public void followBody(final DynamicBody body, final float offsetX,
			final float offsetY, final float offsetZ) {
		followBody = body;
		followOffset.set(offsetX, offsetY, offsetZ);
	}

	@Override
	public void prepareUpdatePosition(final IUpdateInfo uInfo) {
		if (followBody != null) {
//			orientation.set(followBody.getOrientation());
			orientation.rotateV(finalFollowOffset, followOffset);
			final Vector3f h = aabb.getHalfsize();
			h.setScale(finalFollowOffset, 0.5F);
			final Vector3f fp = followBody.getPosition();
			aabb.getPosition().setAdd(fp, h);
			h.abs();
			position.setAdd(fp, finalFollowOffset);
		}
		super.prepareUpdatePosition(uInfo);
	}

	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		if (followBody != null) {
			position.setAdd(followBody.getPosition(), finalFollowOffset);
			aabb.setPosition(position);
		}
	}
}
