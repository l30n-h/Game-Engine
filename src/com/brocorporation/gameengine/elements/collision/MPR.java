package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.elements.collision.Simplex.Element;
import com.brocorporation.gameengine.utils.Distance;
import com.brocorporation.gameengine.utils.Vector3f;

public class MPR {

	protected final static int MAX_ITERATIONS = 34;
	protected final static float CollideEpsilon = 1e-4f;
	protected final static float CollideEpsilon_2 = CollideEpsilon
			* CollideEpsilon;

	protected final static Vector3f temp1 = new Vector3f();
	protected final static Vector3f temp2 = new Vector3f();
	protected final static Vector3f temp3 = new Vector3f();
	protected final static Simplex simplex = new Simplex();
	protected final static Element e0 = new Element();

	protected static Element e1, e2, e3, e4;

	protected static Vector3f dir = new Vector3f();

	public static boolean intersects(IShape shape1, IShape shape2) {
		initElementsFromSimplex(simplex);
		final int res = discoverPortal(shape1, shape2);
		if (res < 0)
			return false;
		if (res > 0)
			return true;
		return refinePortal(shape1, shape2);
	}

	public static boolean intersects(Contact contact, IShape shape1,
			IShape shape2) {
		if (debug) {
			out = false;
			bu.setLength(0);
			Vector3f[] v1 = ((Convex) shape1).getVertices();
			Vector3f[] v2 = ((Convex) shape2).getVertices();
			for (int i = 0; i < v1.length; i++) {
				for (int j = 0; j < v2.length; j++) {
					temp1.setSubtract(v1[i], v2[j]);
					bu.append("p" + i + "" + j + ":Punkt("
							+ temp1.toString().replaceAll("E", "*10^") + ")\n");
				}
			}
			bu.append("\n\n");
		}
		initElementsFromSimplex(simplex);
		final int res = discoverPortal(shape1, shape2);
		if (res < 0) {
			return false;
		} else if (res == 1) {
			findPenetrationTouch(contact, shape1, shape2);
		} else if (res == 2) {
			findPenetrationSegment(contact);
		} else if (res == 0) {
			if (!refinePortal(shape1, shape2)) {
				return false;
			}
			findPenetration(contact, shape1, shape2);

		}
		return true;
	}

	public static void initElementsFromSimplex(Simplex s) {
		e1 = s.get(0);
		e2 = s.get(1);
		e3 = s.get(2);
		e4 = s.get(3);
	}

	protected static void getOriginRayDirection(IShape shape1, IShape shape2) {
		// TODO editable
		 e0.v.setSubtract(e0.pA.set(shape1.getPosition()),
		 e0.pB.set(shape2.getPosition()));
//		AABB.getDistance(e0.v, shape2.getAABB(), shape1.getAABB());
		if (e0.v.isZero()) {
			e0.v.set(0.00001f, 0, 0);
		}
		if (debug) {
			bu.append("discover\n");
			bu.append("v0:Punkt(" + e0.v + ")\n");
		}
	}

	protected static byte discoverPortal(IShape shape1, IShape shape2) {
		getOriginRayDirection(shape1, shape2);
		MinkowskiDifference.getMinSupport(e1, shape1, shape2, e0.v);
		if (e1.v.dot(e0.v) >= 0)
			return -1;
		dir.setCross(e0.v, e1.v);
		if (dir.isZero()) {
			return 2;
		}
		MinkowskiDifference.getMaxSupport(e2, shape1, shape2, dir);
		if (e2.v.dot(dir) <= 0)
			return -1;
		temp1.setSubtract(e1.v, e0.v);
		temp2.setSubtract(e2.v, e0.v);
		dir.setCross(temp1, temp2);
		if (dir.dot(e0.v) > 0) {
			final Element e = e1;
			e1 = e2;
			e2 = e;
			dir.invert();
			if (debug)
				bu.append("swap 1<->2\n");
		}
		while (true) {
			MinkowskiDifference.getMaxSupport(e3, shape1, shape2, dir);
			if (debug) {
				bu.append("v1:Punkt(" + e1.v + ")\n");
				bu.append("v2:Punkt(" + e2.v + ")\n");
				bu.append("v3:Punkt(" + e3.v + ")\n");
				bu.append("d:Vektor(" + dir + ")\n\n");
			}
			if (e3.v.dot(dir) <= 0)
				return -1;
			if (temp1.setCross(e1.v, e3.v).dot(e0.v) < 0) {
				set(e2, e3);
			} else if (temp1.setCross(e3.v, e2.v).dot(e0.v) < 0) {
				set(e1, e3);
			} else {
				return 0;
			}
			temp1.setSubtract(e1.v, e0.v);
			temp2.setSubtract(e2.v, e0.v);
			dir.setCross(temp1, temp2);
		}
	}

	protected static boolean refinePortal(IShape shape1, IShape shape2) {
		if (debug)
			bu.append("refine\n");
		while (true) {
			portalDir(dir);
			if (debug) {
				bu.append("v1:Punkt(" + e1.v + ")\n");
				bu.append("v2:Punkt(" + e2.v + ")\n");
				bu.append("v3:Punkt(" + e3.v + ")\n");
			}
			;
			if (dir.dot(e1.v) >= 0)
				return true;
			MinkowskiDifference.getMaxSupport(e4, shape1, shape2, dir);
			if (debug) {
				bu.append("v4:Punkt(" + e4.v + ")\n");
				bu.append("d:Vektor(" + dir + ")\n\n");
			}
			if (dir.dot(e4.v) < 0 || portalReachTolerance(e4, dir)) {
				return false;
			}
			expandPortal(e4);
		}
	}

	public static StringBuilder bu = new StringBuilder();
	static boolean out = false;
	static boolean debug = false;

	protected static void findPenetration(Contact contact, IShape shape1,
			IShape shape2) {
		if (debug)
			bu.append("penetration\n");
		int iterations = 0;
		while (true) {
			portalDir(dir);
			MinkowskiDifference.getMaxSupport(e4, shape1, shape2, dir);
			if (debug) {
				bu.append("v1:Punkt(" + e1.v + ")\n");
				bu.append("v2:Punkt(" + e2.v + ")\n");
				bu.append("v3:Punkt(" + e3.v + ")\n");
				bu.append("v4:Punkt(" + e4.v + ")\n");
				bu.append("d:Vektor(" + dir + ")\n\n");
			}
			if (portalReachTolerance(e4, dir) || iterations > MAX_ITERATIONS) {
				break;
			}
			expandPortal(e4);

			iterations++;
		}
		final Vector3f n = contact.getNormal();
		n.setNorm(dir);
		float d = e1.v.dot(n);
		if (d <= CollideEpsilon/* _2 */) {
			findPenetrationTouch(contact, shape1, shape2);
		} else {
			// d = (float) Math.sqrt(d);
			contact.setDistance(-d);
			// n.norm();
			findPos(contact.getPointA(), shape1, shape2, n);

			if (debug) {
				bu.append("n : " + n + "\t" + d + "\n");
				bu.append(iterations + "____________________________________\n");
				if (out || d > 0.1 || iterations > 5)
					System.out.println(bu);
			}
		}
	}

	protected static void findPenetrationS(Contact c, IShape shape1,
			IShape shape2) {
		for (int i = 0; i < MAX_ITERATIONS; i++) {
			portalDir(dir);
			MinkowskiDifference.getMaxSupport(e4, shape1, shape2, dir);
			if (portalReachTolerance(e4, dir))
				break;
			expandPortal(e4);
		}
		final Vector3f n = c.getNormal();
		float d = Distance.distanceToTriangle(n, e1.v, e2.v, e3.v);
		if (d <= CollideEpsilon_2) {
			findPenetrationTouch(c, shape1, shape2);
		} else {
			d = (float) Math.sqrt(d);
			c.setDistance(-d);
			n.norm();
			findPos(c.getPointA(), shape1, shape2, n);
		}
		return;
	}

	protected static void findPenetrationTouch(Contact c, IShape shape1,
			IShape shape2) {
		c.setDistance(0);
		c.getNormal().set(0, 0, 0);
		c.getPointA().setAdd(e1.pA, e1.pB).scale(0.5f);
	}

	protected static void findPenetrationSegment(Contact c) {
		c.getPointA().setAdd(e1.pA, e1.pB).scale(0.5f);
		c.getNormal().set(e1.v);
		c.setDistance(-c.getNormal().normLength());
	}

	protected static void findPos(Vector3f pos, IShape shape1, IShape shape2,
			Vector3f normedNormal) {
		float b0 = temp1.setCross(e1.v, e2.v).dot(e3.v);
		float b1 = temp2.setCross(e3.v, e2.v).dot(e0.v);
		float b2 = temp3.setCross(e0.v, e1.v).dot(e3.v);
		float b3 = -temp1.dot(e0.v);

		float sum = b0 + b1 + b2 + b3;

		if (sum <= 0) {
			b1 = -temp2.dot(normedNormal);
			b2 = temp3.setCross(e3.v, e1.v).dot(normedNormal);
			b3 = temp1.dot(normedNormal);
			sum = b1 + b2 + b3;
			final float invHalfSum = 0.5f / sum;
			pos.set(((e1.pA.x + e1.pB.x) * b1 + (e2.pA.x + e2.pB.x) * b2 + (e3.pA.x + e3.pB.x)
					* b3)
					* invHalfSum, ((e1.pA.y + e1.pB.y) * b1
					+ (e2.pA.y + e2.pB.y) * b2 + (e3.pA.y + e3.pB.y) * b3)
					* invHalfSum, ((e1.pA.z + e1.pB.z) * b1
					+ (e2.pA.z + e2.pB.z) * b2 + (e3.pA.z + e3.pB.z) * b3)
					* invHalfSum);
		} else {
			final float invHalfSum = 0.5f / sum;
			pos.set(((e0.pA.x + e0.pB.x) * b0 + (e1.pA.x + e1.pB.x) * b1
					+ (e2.pA.x + e2.pB.x) * b2 + (e3.pA.x + e3.pB.x) * b3)
					* invHalfSum,
					((e0.pA.y + e0.pB.y) * b0 + (e1.pA.y + e1.pB.y) * b1
							+ (e2.pA.y + e2.pB.y) * b2 + (e3.pA.y + e3.pB.y)
							* b3)
							* invHalfSum, ((e0.pA.z + e0.pB.z) * b0
							+ (e1.pA.z + e1.pB.z) * b1 + (e2.pA.z + e2.pB.z)
							* b2 + (e3.pA.z + e3.pB.z) * b3)
							* invHalfSum);
		}
	}

	protected static void expandPortal(Element e) {
		// TODO editable
		temp3.setCross(e.v,e0.v);
		//temp3.setCross(dir, e.v);
		if (e1.v.dot(temp3) > 0) {
			if (e2.v.dot(temp3) > 0) {
				set(e1, e);
			} else {
				set(e3, e);
			}
		} else {
			if (e3.v.dot(temp3) > 0) {
				set(e2, e);
			} else {
				set(e1, e);
			}
		}
	}

	protected static void portalDir(Vector3f dir) {
		temp1.setSubtract(e2.v, e1.v);
		temp2.setSubtract(e3.v, e1.v);
		dir.setCross(temp1, temp2);
	}

	protected static boolean portalReachTolerance(Element e, Vector3f dir) {
		final float min = temp2.setSubtract(e.v, e1.v).dot(dir);
		return min <= 0 || min * min <= CollideEpsilon_2 * dir.dot(dir);
	}

	protected static void set(Element a, Element b) {
		a.v.set(b.v);
		a.pA.set(b.pA);
		a.pB.set(b.pB);
	}
}
