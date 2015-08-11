package com.brocorporation.gameengine.elements.collision;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Iterator;

import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class Manifold {

	protected final static float eps = 0.02f;
	protected final static float eps_2 = eps * eps;

	public static HashMap<Manifold, Manifold> manifolds = new HashMap<Manifold, Manifold>();
	protected static ArrayDeque<Manifold> unused = new ArrayDeque<Manifold>();

	public static Manifold add(StaticBody a, StaticBody b, Contact c) {
		Manifold m = unused.isEmpty() ? new Manifold() : unused.pop();
		m.reset(a, b);
		Manifold m2 = manifolds.get(m);
		if (m2 != null) {
			unused.push(m);
			if (a != m2.bodyA) {
				c.swap();
			}
		} else {
			m2 = m;
			manifolds.put(m2, m2);
		}
		m2.addContact(c);
		return m2;
	}

	public static void update() {
		final Iterator<Manifold> c = manifolds.values().iterator();
		while (c.hasNext()) {
			final Manifold m = c.next();
			m.refreshContactPoints();
			if (m.size() == 0) {
				unused.push(m);
				c.remove();
			}
		}
	}

	protected StaticBody bodyA;
	protected StaticBody bodyB;
	// protected Vector3f normal = new Vector3f();
	protected ManifoldContact[] contacts = { new ManifoldContact(),
			new ManifoldContact(), new ManifoldContact(), new ManifoldContact() };
	protected int size;

	static Vector3f tmpA = new Vector3f();
	static Vector3f tmpB = new Vector3f();

	public Manifold() {

	}

	public void reset(StaticBody bA, StaticBody bB) {
		bodyA = bA;
		bodyB = bB;
		size = 0;
	}

	public StaticBody getBodyA() {
		return bodyA;
	}

	public StaticBody getBodyB() {
		return bodyB;
	}

	public ManifoldContact getContact(int i) {
		return contacts[i];
	}

	public int size() {
		return size;
	}

	static Vector3f a = new Vector3f();
	static Vector3f n = new Vector3f();
	static Vector3f localA = new Vector3f();
	static Vector3f b = new Vector3f();

	public int addContact(Contact c) {
		final Vector3f normal = c.getNormal();
		n.setScale(normal, c.getDistance() / 2);
		a.setSubtract(c.getPointA(), n);
		// bodyA.getAffineTransform().toLocal(localA, a);
		b.setAdd(c.getPointA(), n);
		for (int i = 0; i < size; i++) {
			// tmpA.setSubtract(contacts[i].localA, localA);
			// if (tmpA.dot(tmpA) <= eps_2) {
			// return -1;
			// }
			tmpA.setSubtract(contacts[i].worldA, a);
			if (tmpA.dot(tmpA) <= eps_2) {
				return -1;
			}
			tmpA.setSubtract(contacts[i].worldB, b);
			if (tmpA.dot(tmpA) <= eps_2) {
				return -1;
			}
		}
		int insertIndex = size;
		bodyA.getAffineTransform().toLocal(localA, a);
		if (insertIndex == contacts.length) {
			if (contacts.length >= 4) {
				insertIndex = sortCachedPoints(localA, c.getDistance());
			} else {
				insertIndex = 0;
			}
		} else {
			size++;
		}
		// contacts[insertIndex].reset(c);
		n.add(c.getPointA());
		contacts[insertIndex].distance = c.getDistance();
		contacts[insertIndex].worldA.set(a);
		contacts[insertIndex].worldB.set(n);
		contacts[insertIndex].localA.set(localA);
		bodyB.getAffineTransform().toLocal(contacts[insertIndex].localB, n);
		contacts[insertIndex].normal.set(normal);
		return insertIndex;
	}

	// / sort cached points so most isolated points come first
	private int sortCachedPoints(Vector3f localA, float distance) {
		// calculate 4 possible cases areas, and take biggest area
		// also need to keep 'deepest'
		int maxPenetrationIndex = -1;
		float maxPenetration = distance;
		for (int i = 0; i < 4; i++) {
			if (contacts[i].distance < maxPenetration) {
				maxPenetrationIndex = i;
				maxPenetration = contacts[i].distance;
			}
		}
		int biggestarea = -1;
		float maxVal = -1e30f;
		if (maxPenetrationIndex != 0) {
			tmpA.setSubtract(localA, contacts[1].localA);
			tmpB.setSubtract(contacts[3].localA, contacts[2].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot(tmpB);
			if (res > maxVal) {
				biggestarea = 0;
				maxVal = res;
			}
		}
		tmpA.setSubtract(localA, contacts[0].localA);
		if (maxPenetrationIndex != 1) {
			tmpB.setSubtract(contacts[3].localA, contacts[2].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot(tmpB);
			if (res > maxVal) {
				biggestarea = 1;
				maxVal = res;
			}
		}
		if (maxPenetrationIndex != 2) {
			tmpB.setSubtract(contacts[3].localA, contacts[1].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot(tmpB);
			if (res > maxVal) {
				biggestarea = 2;
				maxVal = res;
			}
		}
		if (maxPenetrationIndex != 3) {
			tmpB.setSubtract(contacts[2].localA, contacts[1].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot(tmpB);
			if (res > maxVal) {
				biggestarea = 3;
			}
		}
		return biggestarea;
	}

	public void refreshContactPoints3() {
		for (int i = size - 1; i >= 0; i--) {
			Vector3f worldA = new Vector3f(), worldB = new Vector3f();
			ManifoldContact manifoldPoint = contacts[i];
			bodyA.getAffineTransform().toWorld(worldA, manifoldPoint.localA);
			bodyB.getAffineTransform().toWorld(worldB, manifoldPoint.localB);
			tmpA.setSubtract(worldA, manifoldPoint.worldA);
			tmpB.setSubtract(worldB, manifoldPoint.worldB);
			if (tmpA.dot(tmpA) > eps_2 || tmpB.dot(tmpB) > eps_2) {
				remove(i);
			}
			float distance = tmpA.dot(manifoldPoint.normal);
			// manifoldPoint.distance = distance;//TODO
		}
	}

	public void refreshContactPoints2() {
		for (int i = size - 1; i >= 0; i--) {
			Vector3f worldA = new Vector3f(), worldB = new Vector3f();
			ManifoldContact manifoldPoint = contacts[i];
			bodyA.getAffineTransform().toWorld(worldA, manifoldPoint.localA);
			bodyB.getAffineTransform().toWorld(worldB, manifoldPoint.localB);
			tmpA.setSubtract(worldB, worldA);
			float distance = tmpA.dot(manifoldPoint.normal);
			if (distance > eps) {
				remove(i);
			} else {
				tmpA.setSubtract(manifoldPoint.worldA, manifoldPoint.worldB);
				if (tmpA.dot(tmpA) - manifoldPoint.distance
						* manifoldPoint.distance > eps_2) {
					remove(i);
				}
			}
			manifoldPoint.distance = distance;// TODO
		}
	}

	public void refreshContactPoints() {
		for (int i = size - 1; i >= 0; i--) {
			ManifoldContact manifoldPoint = contacts[i];
			bodyA.getAffineTransform().toWorld(manifoldPoint.worldA,
					manifoldPoint.localA);
			bodyB.getAffineTransform().toWorld(manifoldPoint.worldB,
					manifoldPoint.localB);
			tmpA.setSubtract(manifoldPoint.worldB, manifoldPoint.worldA);
			manifoldPoint.distance = tmpA.dot(manifoldPoint.normal);

			if (manifoldPoint.distance > eps) {
				remove(i);
			} else {
				tmpA.setSubtract(manifoldPoint.worldA, manifoldPoint.worldB);
				if (tmpA.dot(tmpA) - manifoldPoint.distance
						* manifoldPoint.distance > eps_2) {
					remove(i);
				}
			}
		}
	}

	public void remove(int i) {
		if (i < size && 0 <= i) {
			if (--size > i) {
				final ManifoldContact ei = contacts[i];
				ei.reset();
				while (i < size) {
					contacts[i] = contacts[++i];
				}
				contacts[i] = ei;
			}
		}
	}

	public void clear() {
		size = 0;
	}

	@Override
	public int hashCode() {
		return Collidable.unorderdHashCode(bodyA, bodyB);
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof Manifold) {
			Manifold m = (Manifold) o;
			return (m.bodyA == bodyA && m.bodyB == bodyB)
					|| (m.bodyA == bodyB && m.bodyB == bodyA);
		}
		return false;
	}

	public class ManifoldContact extends Contact {
		// protected Vector3f normal = new Vector3f();//TODO save global
		// protected Vector3f worldA = new Vector3f();
		// protected Vector3f worldB = new Vector3f();
		// protected float distance;
		protected Vector3f localA = new Vector3f();
		protected Vector3f localB = new Vector3f();
		protected float lastImpulseSum;
		protected Vector3f tangent1 = new Vector3f();
		protected Vector3f tangent2 = new Vector3f();

		public void reset() {
			lastImpulseSum = 0;
		}

		public Vector3f getWorldA() {
			return worldA;
		}

		public Vector3f getWorldB() {
			return worldB;
		}

		public Vector3f getNormal() {
			return normal;
		}

		public float getDistance() {
			return distance;
		}

		public void setLastImpulseSum(float l) {
			lastImpulseSum = l;
		}

		public float getLastImpulseSum() {
			return lastImpulseSum;
		}

		public Vector3f getTangent1() {
			return tangent1;
		}

		public Vector3f gettangent1() {
			return tangent2;
		}

		public void calcTangent() {
			if (Math.abs(normal.x) >= 0.57735f) {
				tangent1.set(normal.y, -normal.x, 0);
			} else {
				tangent1.set(0, normal.z, -normal.y);
			}
			tangent1.norm();
			tangent2.setCross(normal, tangent1);
		}
	}
}