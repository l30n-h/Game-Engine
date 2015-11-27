package com.brocorporation.gameengine.elements.collision;

import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Iterator;

import com.brocorporation.gameengine.elements.bodies.StaticBody;
import com.brocorporation.gameengine.utils.Vector3f;

public class Manifold {

	protected final static float eps = 0.04f;
	protected final static float eps_2 = eps * eps;

	private final static Manifold MANIFOLD = new Manifold();
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
	
	public static Manifold getManifold(StaticBody a, StaticBody b){
		MANIFOLD.reset(a, b);
		return manifolds.get(MANIFOLD);
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
	protected final ManifoldContact[] contacts = { new ManifoldContact(),
			new ManifoldContact(), new ManifoldContact(), new ManifoldContact() };
	protected int size;

	static Vector3f tmpA = new Vector3f();
	static Vector3f tmpB = new Vector3f();

	protected float stcFriction, dynFriction;

	public Manifold() {

	}

	public void reset(StaticBody bA, StaticBody bB) {
		bodyA = bA;
		bodyB = bB;
		Material mA = bodyA.getMaterial();
		Material mB = bodyB.getMaterial();
		calculateFriction(mA, mB);

		size = 0;
	}

	public StaticBody getBodyA() {
		return bodyA;
	}

	public StaticBody getBodyB() {
		return bodyB;
	}

	public float getStaticFriction() {
		return stcFriction;
	}

	public float getDynamicFriction() {
		return dynFriction;
	}

	public boolean isFrictionless() {
		return stcFriction == 0;
	}

	public float getRestitution() {
		return Math.max(bodyA.getMaterial().getRestitution(), bodyB
				.getMaterial().getRestitution());
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
			// if (tmpA.dot() <= eps_2) {
			// return -1;
			// }
			if (tmpA.setSubtract(contacts[i].worldA, a).dot() <= eps_2
					|| tmpA.setSubtract(contacts[i].worldB, b).dot() <= eps_2) {
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
		assert(insertIndex!=-1);
		n.add(c.getPointA());
		final ManifoldContact insertContact = contacts[insertIndex];
		insertContact.distance = c.getDistance();
		insertContact.worldA.set(a);
		insertContact.worldB.set(n);
		insertContact.localA.set(localA);
		bodyB.getAffineTransform().toLocal(insertContact.localB, n);
		insertContact.normal.set(normal);
		if (!isFrictionless()) {
			insertContact.calcTangent();
		}
		return insertIndex;
	}

	private int sortCachedPoints(Vector3f localA, float distance) {
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
			float res = tmpB.dot();
			if (res > maxVal) {
				biggestarea = 0;
				maxVal = res;
			}
		}
		tmpA.setSubtract(localA, contacts[0].localA);
		if (maxPenetrationIndex != 1) {
			tmpB.setSubtract(contacts[3].localA, contacts[2].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot();
			if (res > maxVal) {
				biggestarea = 1;
				maxVal = res;
			}
		}
		if (maxPenetrationIndex != 2) {
			tmpB.setSubtract(contacts[3].localA, contacts[1].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot();
			if (res > maxVal) {
				biggestarea = 2;
				maxVal = res;
			}
		}
		if (maxPenetrationIndex != 3) {
			tmpB.setSubtract(contacts[2].localA, contacts[1].localA);
			tmpB.setCross(tmpA, tmpB);
			float res = tmpB.dot();
			if (res > maxVal) {
				biggestarea = 3;
			}
		}
		return biggestarea;
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
				if (tmpA.dot() - manifoldPoint.distance
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

	protected void calculateFriction(Material mA, Material mB) {
		if (mA.getStaticFriction() != 0 || mB.getStaticFriction() != 0) {
			stcFriction = (float) Math.sqrt(mA.getStaticFriction()
					* mA.getStaticFriction() + mB.getStaticFriction()
					* mB.getStaticFriction());
			dynFriction = (float) Math.sqrt(mA.getDynamicFriction()
					* mA.getDynamicFriction() + mB.getDynamicFriction()
					* mB.getDynamicFriction());
		} else {
			stcFriction = 0;
			dynFriction = 0;
		}
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
		protected Vector3f tangent1 = new Vector3f();
		protected Vector3f tangent2 = new Vector3f();
		protected float lastImpulseSum;

		public Manifold getManifold() {
			return Manifold.this;
		}

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

		public Vector3f getTangent2() {
			return tangent2;
		}

		public void calcTangent() {
			Vector3f.computeBasis(normal, tangent1, tangent2);
		}
	}
}