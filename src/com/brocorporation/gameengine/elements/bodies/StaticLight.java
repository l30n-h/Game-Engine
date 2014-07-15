package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.VectorPool;

public class StaticLight extends RigidBody {
	
	protected DynamicBody followBody;
	protected final Vector3f followOffset = new Vector3f();

	public StaticLight(final IShape shape, final float pMass) {
		super(shape, pMass);
	}

	public void followBody(final DynamicBody body, final float offsetX,
			final float offsetY, final float offsetZ) {
		followBody = body;
		followOffset.set(offsetX, offsetY, offsetZ);
	}

	@Override
	public void updatePosition(final IUpdateInfo uInfo) {
		if (followBody != null) {
			final Vector3f translation = affineTransform.getTranslation();
			if (followBody instanceof RigidBody) {
				final Vector3f temp = VectorPool.getVectorV3(false);
				affineTransform.getOrientation().set(((RigidBody) followBody).getOrientation()).rotateV(
						temp, followOffset);
				translation.setAdd(followBody.getPosition(), temp);
				VectorPool.release(temp);
				updateOrientation = true;
			} else {
				translation.setAdd(followBody.getPosition(), followOffset);
			}
		}
	}
}