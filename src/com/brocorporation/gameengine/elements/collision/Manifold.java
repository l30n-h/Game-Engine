package com.brocorporation.gameengine.elements.collision;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Stack;

import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class Manifold {

	protected final static float contactBreakingThreshold = 0.02f;

	public static HashMap<Manifold, Manifold> manifolds = new HashMap<Manifold, Manifold>();
	protected static Stack<Manifold> unused = new Stack<Manifold>();// TODO sync

	public static Manifold add(StaticBody a, StaticBody b, Contact c) {
		Manifold m = unused.isEmpty() ? new Manifold() : unused.pop();
		m.reset(a, b);
		Manifold m2 = manifolds.get(m);
		if (m2 != null) {
			unused.push(m);
		} else {
			m2 = m;
			manifolds.put(m2, m2);
		}
		if (a != m2.bodyA) {
			c.swap();
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
	protected Vector3f normal = new Vector3f();
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

	public Vector3f getNormal() {
		return normal;
	}

	public ManifoldContact getContact(int i) {
		return contacts[i];
	}

	public int size() {
		return size;
	}

	static Vector3f a = new Vector3f();
	static Vector3f b = new Vector3f();
	static Vector3f localA = new Vector3f();

	public int addContact(Contact c) {
		// if (size == 0)
		normal.set(c.getNormal());
		b.setAddScaled(c.getPointA(), normal, c.getDistance() / 2);
		a.setSubtractScaled(b, normal, c.getDistance());
		bodyA.getAffineTransform().toLocal(localA, a);
		int insertIndex = size;
		if (insertIndex == contacts.length) {
			if (contacts.length >= 4) {
				// sort cache so best points come first, based on area
				insertIndex = sortCachedPoints(localA, c.getDistance());
			} else {
				insertIndex = 0;
			}
		} else {
			size++;
		}
		// contacts[insertIndex].reset(c);
		contacts[insertIndex].distance = c.getDistance();
		contacts[insertIndex].worldA.set(a);
		contacts[insertIndex].worldB.set(b);
		contacts[insertIndex].localA.set(localA);
		bodyB.getAffineTransform().toLocal(contacts[insertIndex].localB, b);
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

	private boolean validContactDistance(float distance) {
		return distance <= contactBreakingThreshold;
	}

	// / calculated new worldspace coordinates and depth, and reject points that
	// exceed the collision margin
	public void refreshContactPoints() {
		int i;
		// first refresh worldspace positions and distance
		for (i = size - 1; i >= 0; i--) {
			ManifoldContact manifoldPoint = contacts[i];
			bodyA.getAffineTransform().toWorld(manifoldPoint.worldA,
					manifoldPoint.localA);
			bodyB.getAffineTransform().toWorld(manifoldPoint.worldB,
					manifoldPoint.localB);
			tmpA.setSubtract(manifoldPoint.worldB, manifoldPoint.worldA);
			manifoldPoint.distance = tmpA.dot(normal);
			// manifoldPoint.lifeTime++;
		}
		// then
		float distance2d;
		for (i = size - 1; i >= 0; i--) {
			ManifoldContact manifoldPoint = contacts[i];
			// contact becomes invalid when signed distance exceeds margin
			// (projected on contactnormal direction)
			if (!validContactDistance(manifoldPoint.distance)) {
				remove(i);
			} else {
				// contact also becomes invalid when relative movement
				// orthogonal to normal exceeds margin
				tmpA.setAddScaled(manifoldPoint.worldA, normal,
						manifoldPoint.distance);
				tmpA.setSubtract(manifoldPoint.worldB, tmpA);
				distance2d = tmpA.dot(tmpA);
				if (distance2d > contactBreakingThreshold
						* contactBreakingThreshold) {
					remove(i);
				} else {
					// contact point processed callback
					// if (BulletGlobals.getContactProcessedCallback() != null)
					// {
					// BulletGlobals.getContactProcessedCallback().contactProcessed(manifoldPoint,
					// body0, body1);
					// }
				}
			}
		}
	}

	public void remove(int i) {
		if (i < size && 0 <= i) {
			if (--size > i) {
				final ManifoldContact ei = contacts[i];
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

	public class ManifoldContact {
		protected Vector3f worldA = new Vector3f();
		protected Vector3f worldB = new Vector3f();
		protected Vector3f localA = new Vector3f();
		protected Vector3f localB = new Vector3f();
		// protected Vector3f IpAxN = new Vector3f();
		// protected Vector3f IpBxN = new Vector3f();
		// protected Vector3f IpAxNxpA = new Vector3f();
		// protected Vector3f IpBxNxpB = new Vector3f();
		protected float distance;

		public void reset(ManifoldContact c) {
			distance = c.distance;// TODO
			worldA.setSubtract(c.worldA, bodyA.getPosition());
			worldB.setSubtract(c.worldB, bodyB.getPosition());
			bodyA.getOrientation().rotateInverseV(localA, worldA);
			bodyB.getOrientation().rotateInverseV(localB, worldB);
		}

		// public void calcDependencies() {
		// if (bodyA instanceof RigidBody) {
		// IpAxN.multiplyM3V(
		// ((RigidBody) bodyA).getInverseInertiaTensor(), 0,
		// IpAxN.setCross(worldA, normal));
		// IpAxNxpA.setCross(IpAxNxpA, worldA);
		// }
		// if (bodyB instanceof RigidBody) {
		// IpBxN.multiplyM3V(
		// ((RigidBody) bodyB).getInverseInertiaTensor(), 0,
		// IpBxN.setCross(worldB, normal));
		// IpBxNxpB.setCross(IpBxNxpB, worldB);
		// }
		// }

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
	}
}