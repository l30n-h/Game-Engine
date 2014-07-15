package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.elements.bodies.Actor;
import com.brocorporation.gameengine.elements.bodies.Item;
import com.brocorporation.gameengine.elements.bodies.Plane;
import com.brocorporation.gameengine.elements.bodies.StaticBody;

public class CollisionFilter {

	public static boolean canCollide(final Class<? extends Collidable> class1,
			final Class<? extends Collidable> class2) {
		if (class1 == class2) {
			return !(class1 == Item.class || class1 == Plane.class || class1 == StaticBody.class);
		} else {
			return !((class1 != Actor.class && class2 == Item.class) || (class2 != Actor.class && class1 == Item.class));
		}
	}

	public static boolean canCollide(final Collidable b1, final Collidable b2) {
		return canCollide(b1.getClass(), b2.getClass());
	}
}
