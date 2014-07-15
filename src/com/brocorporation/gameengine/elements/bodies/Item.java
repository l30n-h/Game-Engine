package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.elements.collision.Collidable;
import com.brocorporation.gameengine.elements.collision.IShape;

public class Item extends RigidBody {

	public Item(final IShape shape, final float pMass) {
		super(shape, pMass);
		canActiveCollide(false);
	}

	@Override
	public void onCollide(final Collidable body) {
		if (body instanceof Actor) {
			final float x = ((float) Math.random() * 2 - 1);
			final float y = ((float) Math.random() * 2 - 1);
			final float z = ((float) Math.random() * 36 - 18);
			setPosition(x, y, z);
		}
	}
}
