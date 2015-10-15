package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.elements.collision.Simplex.Element;
import com.brocorporation.gameengine.utils.Distance;
import com.brocorporation.gameengine.utils.Vector3f;

public class GJK {
	private final static int MAX_ITERATIONS = 20;
	private final static float EPSILON = 1e-4f;
	private final static float EPSILON_2 = EPSILON * EPSILON;

	private final static Simplex simplex = new Simplex();
	private final static Vector3f AB = new Vector3f();
	private final static Vector3f AC = new Vector3f();
	private final static Vector3f AD = new Vector3f();

	private final static Vector3f v = new Vector3f();
	private final static Vector3f v0 = new Vector3f();
	private final static Vector3f p = new Vector3f();
	private final static Vector3f w = new Vector3f();
	private static float s, t;

	public static float distance(Contact contact, IShape shape1, IShape shape2) {
		simplex.clear();
		v.setSubtract(shape1.getPosition(), shape2.getPosition());
		Element e = simplex.getNewElement();
		MinkowskiDifference.getMaxSupport(e, shape1, shape2, v);
		simplex.addElement();
		v.set(e.v);
		float d_2 = v.dot();
		int i = 0;
		if (d_2 > EPSILON_2) {
			float d0_2;
			do {
				e = simplex.getNewElement();
				MinkowskiDifference.getMinSupport(e, shape1, shape2, v);
				if (simplex.contains(e.v)) {
					break;
				}
				simplex.addElement();
				v0.set(v);
				d0_2 = d_2;
				d_2 = closestPointToOrigin(v, simplex);
				if (d_2 >= d0_2) {
					// TODO
					// v.set(v0);
					// d_2 = d0_2;
					break;
				}
			} while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS
					&& (d0_2 - 2 * v0.dot(v) + d_2) > EPSILON_2);
		}
		if (d_2 <= EPSILON_2) {
			contact.setDistance(0);
			contact.getNormal().set(0, 0, 0);
			return 0;
		}
		getDistanceClosestPoints(contact, simplex, v);
		d_2 = (float) Math.sqrt(d_2);
		contact.setDistance(d_2);
		contact.getNormal().setScale(v, -1 / d_2);
		return d_2;
	}

	public static boolean intersects(Contact contact, IShape shape1,
			IShape shape2, float r) {
		simplex.clear();
		v.setSubtract(shape1.getPosition(), shape2.getPosition());
		Element e = simplex.getNewElement();
		MinkowskiDifference.getMaxSupport(e, shape1, shape2, v);
		simplex.addElement();
		v.set(e.v);
		float d_2 = v.dot();
		int i = 0;
		boolean checkIntersection = true;
		final float r2 = 2 * r;
		if (d_2 > EPSILON_2) {
			final float r2_2 = r2 * r2;
			float d0_2;
			do {
				e = simplex.getNewElement();
				MinkowskiDifference.getMinSupport(e, shape1, shape2, v);
				if (checkIntersection) {
					final float vw = v.dot(e.v);
					if (vw > 0 && vw * vw > r2_2 * d_2) {
						checkIntersection = false;
					}
				}
				if (simplex.contains(e.v)) {
					break;
				}
				simplex.addElement();
				v0.set(v);
				d0_2 = d_2;
				d_2 = closestPointToOrigin(v, simplex);
				if (d_2 > d0_2) {
					// TODO
					// v.set(v0);
					// d_2 = d0_2;
					break;
				}
			} while (d_2 > EPSILON_2
					&& i++ < MAX_ITERATIONS
					&& (checkIntersection || (d0_2 - 2 * v0.dot(v) + d_2) > EPSILON_2));
		}
		if (checkIntersection) {
			if (d_2 <= EPSILON_2) {
				contact.setDistance(0);
				contact.getNormal().set(0, 0, 0);
				// EPA
				return true;
			} else {
				getIntersectsClosestPoints(contact, simplex, v);
				if (simplex.size() == 3) {
					contact.setDistance(-(r2 - (float) Math.sqrt(Distance
							.distanceToPlane(v, simplex.getV(2),
									simplex.getV(1), simplex.getV(0)))));
					contact.getNormal().setInvert(v).norm();
				} else {
					d_2 = (float) Math.sqrt(d_2);
					contact.setDistance(-(r2 - d_2));
					contact.getNormal().setScale(v, -1 / d_2);
				}
				return true;
			}
		} else {
			getDistanceClosestPoints(contact, simplex, v);
			d_2 = (float) Math.sqrt(d_2);
			contact.setDistance(d_2);
			if (d_2 == 0) {// TODO Problem sollte nicht vorkommen oder? liegt am
							// Tetrahedron aufgefallen mit Sphere
				contact.setDistance(-v.normLength());
				contact.getNormal().set(v);
			} else
				contact.getNormal().setScale(v, -1 / d_2);
			return false;
		}
	}

	public static boolean intersects(final IShape shape1, final Vector3f vertex) {
		simplex.clear();
		Element e = simplex.getNewElement();
		MinkowskiDifference.getMaxSupport(e, shape1, vertex, v);
		simplex.addElement();
		v.set(e.v);
		float d_2 = v.dot();
		int i = 0;
		while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS) {
			e = simplex.getNewElement();
			MinkowskiDifference.getMinSupport(e, shape1, vertex, v);
			if (v.dot(e.v) > 0) {
				return false;
			}
			if (simplex.contains(e.v))
				break;
			simplex.addElement();
			d_2 = closestPointToOrigin(v, simplex);
		}
		return true;
	}

	public static boolean intersects(IShape shape1, IShape shape2) {
		simplex.clear();
		v.setSubtract(shape1.getPosition(), shape2.getPosition());
		Element e = simplex.getNewElement();
		MinkowskiDifference.getMaxSupport(e, shape1, shape2, v);
		simplex.addElement();
		v.set(e.v);
		float d_2 = v.dot();
		int i = 0;
		while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS) {
			e = simplex.getNewElement();
			MinkowskiDifference.getMinSupport(e, shape1, shape2, v);
			if (v.dot(e.v) > 0) {
				return false;
			}
			if (simplex.contains(e.v))
				break;
			simplex.addElement();
			d_2 = closestPointToOrigin(v, simplex);
		}
		return true;
	}

	public static boolean rayCast(RaycastHit hit, Vector3f normal,
			Vector3f from, Vector3f direction, IShape stcShape) {
		simplex.clear();
		float s = 0;
		final Vector3f x = hit.getPoint().set(from);
		simplex.setRef(x);
		v.setSubtract(x, stcShape.getPosition());
		int i = 0;
		float d_2 = v.dot();
		while (d_2 > EPSILON_2) {
			if (i > MAX_ITERATIONS) {
				return false;
			}
			i++;
			stcShape.getMaxAlongDirection(p, v);
			w.setSubtract(x, p);
			final float vdotw = v.dot(w);
			if (vdotw > 0) {
				final float vdotdirection = v.dot(direction);
				if (vdotdirection >= 0) {
					return false;
				} else {
					s -= vdotw / vdotdirection;
					x.setAddScaled(from, direction, s);
					normal.set(v);
				}
			}
			if (!simplex.contains(p))
				simplex.add(p);
			d_2 = closestPointToOrigin(v, simplex);
		}
		hit.setScalar(s);
		return true;
	}

	public static boolean rayCast(RaycastHit hit, Vector3f normal,
			Vector3f from, Vector3f direction, IShape dynShape, IShape stcShape) {
		simplex.clear();
		float s = 0;
		final Vector3f x = hit.getPoint().set(from);
		simplex.setRef(x);
		v.setSubtract(x, v.setSubtract(from, stcShape.getPosition()));
		int i = 0;
		float d_2 = v.dot();
		while (d_2 > EPSILON_2) {
			if (i >= MAX_ITERATIONS) {
				return false;
			}
			i++;
			final Element e = simplex.getNewElement();
			MinkowskiDifference.getMaxSupport(e, stcShape, dynShape, v);
			w.setSubtract(x, e.v);
			final float vdotw = v.dot(w);
			if (vdotw > 0) {
				final float vdotdirection = v.dot(direction);
				if (vdotdirection >= 0) {
					return false;
				} else {
					s -= vdotw / vdotdirection;
					x.setAddScaled(from, direction, s);
					normal.set(v);
				}
			}
			if (!simplex.contains(e.v))
				simplex.addElement();
			d_2 = closestPointToOrigin(v, simplex);
		}
		hit.setScalar(s);
		return true;
	}

	private static float closestPointToLineSegment(Vector3f result,
			Simplex simplex) {
		final Vector3f A = simplex.getV(1);
		final Vector3f B = simplex.getV(0);
		AB.setSubtract(B, A);
		final float ABdotA = AB.dot(A);
		if (ABdotA >= 0) {
			simplex.removeB();
			result.set(A);
		} else {
			final float ABdotAB = -AB.dot();
			if (ABdotA <= ABdotAB) {
				simplex.removeA();
				result.set(B);
			} else {
				s = ABdotA / ABdotAB;
				result.setAddScaled(A, AB, s);
			}
		}
		return result.dot();
	}

	private static float closestPointToTriangle(Vector3f result, Simplex simplex) {
		final Vector3f A = simplex.getV(2);
		final Vector3f B = simplex.getV(1);
		final Vector3f C = simplex.getV(0);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		final float aoab = -A.dot(AB);
		final float aoac = -A.dot(AC);
		if (aoab <= 0 && aoac <= 0) {
			simplex.removeB();
			simplex.removeC();
			result.set(A);
			return result.dot();
		}
		final float boab = -B.dot(AB);
		final float boac = -B.dot(AC);
		if (boab >= 0 && boac <= boab) {
			simplex.removeA();
			simplex.removeC();
			result.set(B);
			return result.dot();
		}
		final float vc = aoab * boac - boab * aoac;
		if (vc <= 0 && aoab >= 0 && boab <= 0) {
			simplex.removeC();
			s = aoab / (aoab - boab);
			result.setAddScaled(A, AB, s);
			return result.dot();
		}
		final float coab = -C.dot(AB);
		final float coac = -C.dot(AC);
		if (coac >= 0 && coab <= coac) {
			simplex.removeA();
			simplex.removeB();
			result.set(C);
			return result.dot();
		}
		final float vb = coab * aoac - aoab * coac;
		if (vb <= 0 && aoac >= 0 && coac <= 0) {
			simplex.removeB();
			s = aoac / (aoac - coac);
			result.setAddScaled(A, AC, s);
			return result.dot();
		}
		final float va = boab * coac - coab * boac;
		final float bacmbab, cabmcac;
		if (va <= 0 && (bacmbab = boac - boab) >= 0
				&& (cabmcac = coab - coac) >= 0) {
			simplex.removeA();
			s = bacmbab / (bacmbab + cabmcac);
			result.setAddScaled(B, AB.setSubtract(C, B), s);
			return result.dot();
		}
		final float denom = 1 / (va + vb + vc);
		s = vb * denom;
		t = vc * denom;
		result.setAddScaled(A, AB, s).addScaled(AC, t);
		return result.dot();
	}

	static Vector3f tmp = new Vector3f();

	private static float closestPointToTetrahedron(Vector3f result,
			Simplex simplex) {
		final Vector3f A = simplex.getV(3);
		final Vector3f B = simplex.getV(2);
		final Vector3f C = simplex.getV(1);
		final Vector3f D = simplex.getV(0);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		AD.setSubtract(D, A);
		float abXacX = AB.y * AC.z - AB.z * AC.y;
		float abXacY = AB.z * AC.x - AB.x * AC.z;
		float abXacZ = AB.x * AC.y - AB.y * AC.x;
		float dO = AD.x * abXacX + AD.y * abXacY + AD.z * abXacZ;
		if (Math.abs(dO) <= EPSILON_2) {
			// TODO degenerated lösche nur damit contact points berechnet werden
			// können
			simplex.removeA();
			return result.dot();
		}
		float dD = -A.x * abXacX - A.y * abXacY - A.z * abXacZ;
		final float signD = Math.signum(dD);
		if (signD == 0) {
			// triangle ABC distance 0
			simplex.removeD();
			result.set(0, 0, 0);
			return 0;
		}
		float aXadX = A.y * AD.z - A.z * AD.y;
		float aXadY = A.z * AD.x - A.x * AD.z;
		float aXadZ = A.x * AD.y - A.y * AD.x;
		float dC = -AB.x * aXadX - AB.y * aXadY - AB.z * aXadZ;
		final float signC = Math.signum(dC);
		if (signC == 0) {
			// triangle ABD distance 0
			simplex.removeC();
			result.set(0, 0, 0);
			return 0;
		}
		float dB = AC.x * aXadX + AC.y * aXadY + AC.z * aXadZ;
		final float signB = Math.signum(dB);
		if (signB == 0) {
			// triangle ACD distance 0
			simplex.removeB();
			result.set(0, 0, 0);
			return 0;
		}
		final float dOsign = Math.signum(dO);
		final boolean samedOdB = dOsign == signB;
		final boolean samedOdC = dOsign == signC;
		final boolean samedOdD = dOsign == signD;
		if (samedOdB && samedOdC && samedOdD) {
			// inside tetrahedron
			return 0;
		}
		if (dOsign != Math.signum(dO - dD - dC - dB)) {
			// TODO dOsing != dAsign remove A cause Origin lies outside BCD
			simplex.removeA();
			return result.dot();
		}
		byte rem = 0;
		float bestsqD = Float.POSITIVE_INFINITY;
		float d;
		final float aoab = -A.dot(AB);
		final float aoac = -A.dot(AC);
		final float aoad = -A.dot(AD);
		if (!samedOdD) {// distance to ABC
			if (aoab <= 0 && aoac <= 0) {
				bestsqD = A.dot();
				result.set(A);
				rem = 14;
			} else {
				final float boab = -B.dot(AB);
				final float boac = -B.dot(AC);
				if (boab >= 0 && boac <= boab) {
					bestsqD = B.dot();
					result.set(B);
					rem = 13;
				} else {
					final float vc = aoab * boac - boab * aoac;
					if (vc <= 0 && aoab >= 0 && boab <= 0) {
						s = aoab / (aoab - boab);
						tmp.setAddScaled(A, AB, s);
						bestsqD = tmp.dot();
						result.set(tmp);
						rem = 12;
					} else {
						final float coab = -C.dot(AB);
						final float coac = -C.dot(AC);
						if (coac >= 0 && coab <= coac) {
							bestsqD = C.dot();
							result.set(C);
							rem = 11;
						} else {
							final float vb = coab * aoac - aoab * coac;
							if (vb <= 0 && aoac >= 0 && coac <= 0) {
								s = aoac / (aoac - coac);
								tmp.setAddScaled(A, AC, s);
								bestsqD = tmp.dot();
								result.set(tmp);
								rem = 10;
							} else {
								final float va = boab * coac - coab * boac;
								final float bacmbab, cabmcac;
								if (va <= 0 && (bacmbab = boac - boab) >= 0
										&& (cabmcac = coab - coac) >= 0) {
									s = bacmbab / (bacmbab + cabmcac);
									tmp.setAddScaled(B, tmp.setSubtract(C, B),
											s);
									bestsqD = tmp.dot();
									result.set(tmp);
									rem = 9;
								} else {
									final float denom = 1 / (va + vb + vc);
									s = vb * denom;
									t = vc * denom;
									tmp.setAddScaled(A, AB, s).addScaled(AC, t);
									bestsqD = tmp.dot();
									result.set(tmp);
									rem = 8;
								}
							}
						}
					}
				}
			}
		}
		if (!samedOdC) {// distance to ABD
			if (aoab <= 0 && aoad <= 0) {
				d = A.dot();
				if (d < bestsqD) {
					bestsqD = d;
					result.set(A);
					rem = 14;
				}
			} else {
				final float boab = -B.dot(AB);
				final float boad = -B.dot(AD);
				if (boab >= 0 && boad <= boab) {
					d = B.dot();
					if (d < bestsqD) {
						bestsqD = d;
						result.set(B);
						rem = 13;
					}
				} else {
					final float vd = aoab * boad - boab * aoad;
					if (vd <= 0 && aoab >= 0 && boab <= 0) {
						float ss = aoab / (aoab - boab);
						tmp.setAddScaled(A, AB, ss);
						d = tmp.dot();
						if (d < bestsqD) {
							s = ss;
							bestsqD = d;
							result.set(tmp);
							rem = 12;
						}
					} else {
						final float doab = -D.dot(AB);
						final float doad = -D.dot(AD);
						if (doad >= 0 && doab <= doad) {
							d = D.dot();
							if (d < bestsqD) {
								bestsqD = d;
								result.set(D);
								rem = 7;
							}
						} else {
							final float vb = doab * aoad - aoab * doad;
							if (vb <= 0 && aoad >= 0 && doad <= 0) {
								float ss = aoad / (aoad - doad);
								tmp.setAddScaled(A, AD, ss);
								d = tmp.dot();
								if (d < bestsqD) {
									s = ss;
									bestsqD = d;
									result.set(tmp);
									rem = 6;
								}
							} else {
								final float va = boab * doad - doab * boad;
								final float badmbab, dabmdad;
								if (va <= 0 && (badmbab = boad - boab) >= 0
										&& (dabmdad = doab - doad) >= 0) {
									float ss = badmbab / (badmbab + dabmdad);
									tmp.setAddScaled(B, tmp.setSubtract(D, B),
											ss);
									d = tmp.dot();
									if (d < bestsqD) {
										s = ss;
										bestsqD = d;
										result.set(tmp);
										rem = 5;
									}
								} else {
									final float denom = 1 / (va + vb + vd);
									float ss = vb * denom;
									float tt = vd * denom;
									tmp.setAddScaled(A, AB, ss).addScaled(AD,
											tt);
									d = tmp.dot();
									if (d < bestsqD) {
										s = ss;
										t = tt;
										bestsqD = d;
										result.set(tmp);
										rem = 4;
									}
								}
							}
						}
					}
				}
			}
		}
		if (!samedOdB) {// distance to ACD
			if (aoac <= 0 && aoad <= 0) {
				d = A.dot();
				if (d < bestsqD) {
					bestsqD = d;
					result.set(A);
					rem = 14;
				}
			} else {
				final float coac = -C.dot(AC);
				final float coad = -C.dot(AD);
				if (coac >= 0 && coad <= coac) {
					d = C.dot();
					if (d < bestsqD) {
						bestsqD = d;
						result.set(C);
						rem = 11;
					}
				} else {
					final float vd = aoac * coad - coac * aoad;
					if (vd <= 0 && aoac >= 0 && coac <= 0) {
						float ss = aoac / (aoac - coac);
						tmp.setAddScaled(A, AC, ss);
						d = tmp.dot();
						if (d < bestsqD) {
							s = ss;
							bestsqD = d;
							result.set(tmp);
							rem = 10;
						}
					} else {
						final float doac = -D.dot(AC);
						final float doad = -D.dot(AD);
						if (doad >= 0 && doac <= doad) {
							d = D.dot();
							if (d < bestsqD) {
								bestsqD = d;
								result.set(D);
								rem = 7;
							}
						} else {
							final float vc = doac * aoad - aoac * doad;
							if (vc <= 0 && aoad >= 0 && doad <= 0) {
								float ss = aoad / (aoad - doad);
								tmp.setAddScaled(A, AD, ss);
								d = tmp.dot();
								if (d < bestsqD) {
									s = ss;
									bestsqD = d;
									result.set(tmp);
									rem = 6;
								}
							} else {
								final float va = coac * doad - doac * coad;
								final float cadmcac, dacmdad;
								if (va <= 0 && (cadmcac = coad - coac) >= 0
										&& (dacmdad = doac - doad) >= 0) {
									float ss = cadmcac / (cadmcac + dacmdad);
									tmp.setAddScaled(C, tmp.setSubtract(D, C),
											ss);
									d = tmp.dot();
									if (d < bestsqD) {
										s = ss;
										bestsqD = d;
										result.set(tmp);
										rem = 3;
									}
								} else {
									final float denom = 1 / (va + vc + vd);
									float ss = vc * denom;
									float tt = vd * denom;
									tmp.setAddScaled(A, AC, ss).addScaled(AD,
											tt);
									d = tmp.dot();
									if (d < bestsqD) {
										s = ss;
										t = tt;
										bestsqD = d;
										result.set(tmp);
										rem = 2;
									}
								}
							}
						}
					}
				}
			}
		}
		// smallest distance wins
		if ((rem & 1) == 1)
			simplex.removeA();
		if ((rem & 2) == 2)
			simplex.removeB();
		if ((rem & 4) == 4)
			simplex.removeC();
		if ((rem & 8) == 8)
			simplex.removeD();
		return bestsqD;
	}

	private static float closestPointToOrigin(Vector3f result, Simplex simplex) {
		final int size = simplex.size();
		if (size == 1) {
			result.set(simplex.getV(0));
			return result.dot();
		} else if (size == 2) {
			return closestPointToLineSegment(result, simplex);
		} else if (size == 3) {
			return closestPointToTriangle(result, simplex);
		} else {
			return closestPointToTetrahedron(result, simplex);
		}
	}

	public static void getDistanceClosestPoints(Contact c, Simplex simplex,
			Vector3f normal) {
		if (simplex.size() == 1) {
			final Element e0 = simplex.get(0);
			c.getPointA().set(e0.pA);
			c.getPointB().set(e0.pB);
		} else if (simplex.size() == 2) {
			final Element eA = simplex.get(0);
			final Element eB = simplex.get(1);
			float r2 = 1 - s;
			c.getPointA().setScale(eA.pA, s).addScaled(eB.pA, r2);
			if (normal == null) {
				c.getPointB().set(eA.pB.scale(s).addScaled(eB.pB, r2));
			} else
				c.getPointB().setSubtract(c.getPointA(), normal);
		} else if (simplex.size() == 3) {
			final Element eA = simplex.get(0);
			final Element eB = simplex.get(1);
			final Element eC = simplex.get(2);
			float r3 = 1 - t - s;
			c.getPointA().setScale(eA.pA, t).addScaled(eB.pA, s)
					.addScaled(eC.pA, r3);
			if (normal == null) {
				c.getPointB()
						.set(eA.pB.scale(t).addScaled(eB.pB, s)
								.addScaled(eC.pB, r3));
			} else
				c.getPointB().setSubtract(c.getPointA(), normal);
		}
	}

	public static void getIntersectsClosestPoints(Contact c, Simplex simplex,
			Vector3f normal) {
		if (simplex.size() == 1) {
			final Element e0 = simplex.get(0);
			c.getPointA().set(e0.pA);
			c.getPointB().set(c.getPointA());
		} else if (simplex.size() == 2) {
			final Element eA = simplex.get(0);
			final Element eB = simplex.get(1);
			float r2 = 1 - s;
			c.getPointA().setScale(eA.pA, s).addScaled(eB.pA, r2);
			c.getPointB().set(c.getPointA().subtractScaled(normal, 0.5f));
		} else if (simplex.size() == 3) {
			final Element eA = simplex.get(0);
			final Element eB = simplex.get(1);
			final Element eC = simplex.get(2);
			float r3 = 1 - t - s;
			c.getPointA().setScale(eA.pA, t).addScaled(eB.pA, s)
					.addScaled(eC.pA, r3);
			c.getPointB().set(c.getPointA().subtractScaled(normal, 0.5f));
		}
	}
}
