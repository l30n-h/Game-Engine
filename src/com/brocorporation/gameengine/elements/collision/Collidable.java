package com.brocorporation.gameengine.elements.collision;

import java.util.BitSet;

public class Collidable {

	protected final BitSet collisionGroups = new BitSet();
	protected final BitSet collisionMask = new BitSet();
	protected IShape shape;

	public Collidable(IShape pShape) {
		shape = pShape;
		isGroupMember(0, true);
		canCollideWithGroup(0, true);
	}

	public void canActiveCollide(boolean active) {
		if (!active) {
			collisionGroups.clear();
		}
	}

	public void canPassiveCollide(boolean passive) {
		if (!passive) {
			collisionMask.clear();
		}
	}

	public boolean canActiveCollide() {
		return !collisionGroups.isEmpty();
	}

	public boolean canPassiveCollide() {
		return !collisionMask.isEmpty();
	}
	
	public boolean canActiveCollide(Collidable c) {
		return collisionGroups.intersects(c.collisionMask);
	}

	public boolean canPassiveCollide(Collidable c) {
		return collisionMask.intersects(c.collisionGroups);
	}

	public boolean canCollide(Collidable c) {
		return collisionGroups.intersects(c.collisionMask)
				|| collisionMask.intersects(c.collisionGroups);
	}

	public boolean isGroupMember(int group) {
		return collisionGroups.get(group);
	}

	public void isGroupMember(int group, boolean member) {
		collisionGroups.set(group, member);
	}

	public boolean canCollideWithGroup(int group) {
		return collisionMask.get(group);
	}

	public void canCollideWithGroup(int group, boolean canCollide) {
		collisionMask.set(group, canCollide);
	}

	public void onCollide(final Collidable body) {

	}

	public IShape getShape() {
		return shape;
	}
}
