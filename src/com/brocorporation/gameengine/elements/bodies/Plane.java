package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.elements.collision.Convex;
import com.brocorporation.gameengine.elements.collision.GJK;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.elements.collision.RaycastHit;
import com.brocorporation.gameengine.utils.Vector3f;

public class Plane extends StaticBody {

	public final static byte FRONT = 0;
	public final static byte BACK = 1;
	public final static byte INSIDE = 2;

	protected final Vector3f normal;
	protected final float distance;

	final static Vector3f temp = new Vector3f();

	public Plane(final Vector3f pNormal, final Convex pBounds) {
		super(pBounds);
		normal = pNormal;
		distance = pBounds.getVertices()[0].dot(normal);
//		Vector3f h = getShape().getAABB().getHalfsize();//TODO mpr cheat
//		getPosition().subtractScaled(normal, Math.max(h.x,Math.max(h.y, h.z)));
	}

	public byte getSide(final Vector3f position) {
		final float p = position.dot(normal);
		if (p > distance)
			return FRONT;
		else if (p < distance)
			return BACK;
		return INSIDE;
	}

	public boolean getCollisionPointPD(final RaycastHit hit,
			final Vector3f fromPoint, final Vector3f direction,
			final float min, final float max) {
		final float dot = normal.dot(direction);
		if (dot != 0) {
			final float s = (distance - normal.dot(fromPoint)) / dot;
			if (s >= min && s <= max) {
				hit.setScalar(s);
				hit.getPoint().setAddScaled(fromPoint, direction, s);
				return true;
			}
		}
		return false;
	}

	public boolean getCollisionPointPP(final RaycastHit hit,
			final Vector3f fromPoint, final Vector3f toPoint, final float min,
			final float max) {
		temp.setSubtract(toPoint, fromPoint);
		return getCollisionPointPD(hit, fromPoint, temp, min, max);
	}

	public boolean intersectsRay(final RaycastHit hit,
			final Vector3f fromPoint, final Vector3f dir, final float min,
			final float max) {
		final float dot = normal.dot(dir);
		if (dot != 0) {
			final float s = (distance - fromPoint.dot(normal)) / dot;
			final Vector3f h;
			final Vector3f p;
			if (s >= min
					&& s <= max
					&& Math.abs((temp.x = fromPoint.x + dir.x * s)
							- (p = shape.getAABB().getPosition()).x) <= (h = shape
							.getAABB().getHalfsize()).x
					&& Math.abs((temp.y = fromPoint.y + dir.y * s) - p.y) <= h.y
					&& Math.abs((temp.z = fromPoint.z + dir.z * s) - p.z) <= h.z
					&& GJK.intersects(shape, temp)) {
				hit.setScalar(s);
				hit.getPoint().set(temp);
				return true;
			}
		}
		return false;
	}

	public Vector3f getNormal() {
		return normal;
	}

	public float getDistance() {
		return distance;
	}
	
	public float getDistance(Vector3f vertex) {
		return vertex.dot(normal) - distance;
	}

	public float getDistance(final IShape sm) {
		sm.getMinAlongDirection(temp, normal);
		return temp.dot(normal) - distance;
	}
}
