package com.brocorporation.gameengine.utils;

import com.brocorporation.gameengine.elements.collision.Simplex;


public class Distance {

	protected final static Vector3f AP = new Vector3f();
	protected final static Vector3f AB = new Vector3f();
	protected final static Vector3f AC = new Vector3f();
	protected final static Vector3f CB = new Vector3f();
	protected final static Vector3f temp = new Vector3f();

	public static float closestPointToLineSegment(Vector3f result, Vector3f A,
			Vector3f B) {
		AB.setSubtract(B, A);
		final float ABdotA = AB.dot(A);
		if (ABdotA >= 0) {
			result.set(A);
		} else {
			final float ABdotAB = AB.dot(AB);
			if (-ABdotA >= ABdotAB) {
				result.set(B);
			} else {
				result.setAddScaled(A, AB, -ABdotA / ABdotAB);
			}
		}
		return result.dot(result);
	}

	public static float distanceToPlane(Vector3f result, Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		result.setCross(AB, AC);
		final float an = A.dot(result);
		if (an < 0) {
			result.invert();
		}
		return (an * an) / result.dot(result);
	}
	
	public static float closestPointToTriangle(Vector3f result, Simplex simplex) {
		final Vector3f A = simplex.getV(2);
		final Vector3f B = simplex.getV(1);
		final Vector3f C = simplex.getV(0);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		final float aab = -A.dot(AB);
		final float aac = -A.dot(AC);float s,t;
		if (aab <= 0 && aac <= 0) {
			simplex.removeB();
			simplex.removeC();
			result.set(A);
			return result.dot(result);
		}
		final float bab = -B.dot(AB);
		final float bac = -B.dot(AC);
		if (bab >= 0 && bac <= bab) {
			simplex.removeA();
			simplex.removeC();
			result.set(B);
			return result.dot(result);
		}
		final float vc = aab * bac - bab * aac;
		if (vc <= 0 && aab >= 0 && bab <= 0) {
			simplex.removeC();
			s = aab / (aab - bab);
			result.setAddScaled(A, AB, s);
			return result.dot(result);
		}
		final float cab = -C.dot(AB);
		final float cac = -C.dot(AC);
		if (cac >= 0 && cab <= cac) {
			simplex.removeA();
			simplex.removeB();
			result.set(C);
			return result.dot(result);
		}
		final float vb = cab * aac - aab * cac;
		if (vb <= 0 && aac >= 0 && cac <= 0) {
			simplex.removeB();
			s = aac / (aac - cac);
			result.setAddScaled(A, AC, s);
			return result.dot(result);
		}
		final float va = bab * cac - cab * bac;
		final float bacmbab, cabmcac;
		if (va <= 0 && (bacmbab = bac - bab) >= 0 && (cabmcac = cab - cac) >= 0) {
			simplex.removeA();
			s = bacmbab / (bacmbab + cabmcac);
			result.setAddScaled(B, AB.setSubtract(C, B), s);
			return result.dot(result);
		}
		final float denom = 1 / (va + vb + vc);
		s = vb * denom;
		t = vc * denom;
		result.setAddScaled(A, AB, s).addScaled(AC, t);
		return result.dot(result);
	}

	public static float closestPointToTriangle(Vector3f result, Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		final float abab = AB.dot(AB);
		final float abac = AB.dot(AC);
		final float acac = AC.dot(AC);
		final float aab = A.dot(AB);
		final float aac = A.dot(AC);
		final float det = Math.abs(abab * acac - abac * abac);
		if (det == 0)
			return Float.NaN;
		float s = abac * aac - acac * aab;
		float t = abac * aab - abab * aac;
		if (s + t <= det) {
			if (s < 0) {
				if (t < 0) {
					if (aab < 0) {
						if (-aab >= abab) {
							result.set(B);
						} else {
							s = -aab / abab;
							result.setAddScaled(A, AB, s);
						}
					} else {
						if (aac >= 0) {
							result.set(A);
						} else if (-aac >= acac) {
							result.set(C);
						} else {
							t = -aac / acac;
							result.setAddScaled(A, AC, t);
						}
					}
				} else {
					if (aac >= 0) {
						result.set(A);
					} else if (-aac >= acac) {
						result.set(C);
					} else {
						t = -aac / acac;
						result.setAddScaled(A, AC, t);
					}
				}
			} else if (t < 0) {
				if (aab >= 0) {
					result.set(A);
				} else if (-aab >= abab) {
					result.set(B);
				} else {
					s = -aab / abab;
					result.setAddScaled(A, AB, s);
				}
			} else {
				float invDet = 1 / det;
				s *= invDet;
				t *= invDet;
				result.setAddScaled(A, AB, s).addScaled(AC, t);
			}
		} else {
			if (s < 0) {
				final float tmp0 = abac + aab;
				final float tmp1 = acac + aac;
				if (tmp1 > tmp0) {
					final float numer = tmp1 - tmp0;
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
						result.set(B);
					} else {
						s = 1 - numer / denom;
						result.setAddScaled(B, AB.setSubtract(C, B), s);
					}
				} else {
					if (tmp1 <= 0) {
						result.set(C);
					} else if (aac >= 0) {
						result.set(A);
					} else {
						t = -aac / acac;
						result.setAddScaled(A, AC, t);
					}
				}
			} else if (t < 0) {
				final float tmp0 = abac + aac;
				final float tmp1 = abab + aab;
				if (tmp1 > tmp0) {
					final float numer = tmp1 - tmp0;
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
						result.set(C);
					} else {
						s = numer / denom;
						result.setAddScaled(B, AB.setSubtract(C, B), s);
					}
				} else {
					if (tmp1 <= 0) {
						result.set(B);
					} else if (aab >= 0) {
						result.set(A);
					} else {
						s = -aab / abab;
						result.setAddScaled(A, AB, s);
					}
				}
			} else {
				final float numer = acac + aac - abac - aab;
				if (numer <= 0) {
					result.set(C);
				} else {
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
						result.set(B);
					} else {
						s = 1 - numer / denom;
						result.setAddScaled(B, AB.setSubtract(C, B), s);
					}
				}
			}
		}
		return result.dot(result);
	}

	public static int removeFromTriangle(Vector3f P, Vector3f A, Vector3f B,
			Vector3f C) {
		AP.setSubtract(P, A);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		CB.setSubtract(B, C);
		dir.setCross(AB, AC);
		int front = 0;
		if (temp.setCross(AB, dir).dot(AP) > 0) {
			front += 1;
		}
		if (temp.setCross(dir, AC).dot(AP) > 0) {
			front += 2;
		}
		if (temp.setCross(dir, CB).dot(B) < 0) {
			front += 4;
		}
		if (front == 0) {
			return 0;
		} else if (front == 1) {
			return 3;
		} else if (front == 2) {
			return 2;
		} else if (front == 3) {
			if (AP.dot(AB) > 0)
				return 3;
			return 2;
		} else if (front == 4) {
			return 1;
		} else if (front == 5) {
			if (B.dot(CB) > 0)
				return 1;
			return 3;
		} else {
			if (C.dot(AC) > 0)
				return 2;
			return 1;
		}
	}

	public static float distanceToTriangle(Vector3f result, Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		CB.setSubtract(B, C);
		result.setCross(AB, AC);
		int front = 0;
		if (temp.setCross(AB, result).dot(A) < 0) {
			front += 1;
		}
		if (temp.setCross(result, AC).dot(A) < 0) {
			front += 2;
		}
		if (temp.setCross(result, CB).dot(B) < 0) {
			front += 4;
		}
		if (front == 0) {
			final float an = A.dot(result);
			if (an < 0) {
				result.invert();
			}
			return (an * an) / result.dot(result);
		} else if (front == 1) {
			return closestPointToLineSegment(result, A, B);
		} else if (front == 2) {
			return closestPointToLineSegment(result, A, C);
		} else if (front == 3) {
			if (A.dot(AB) < 0)
				return closestPointToLineSegment(result, A, B);
			return closestPointToLineSegment(result, A, C);
		} else if (front == 4) {
			return closestPointToLineSegment(result, B, C);
		} else if (front == 5) {
			if (B.dot(CB) > 0)
				return closestPointToLineSegment(result, B, C);
			return closestPointToLineSegment(result, A, B);
		} else {
			if (C.dot(AC) > 0)
				return closestPointToLineSegment(result, A, C);
			return closestPointToLineSegment(result, B, C);
		}
	}

	static Vector3f dir = new Vector3f();

	public static boolean intersectsLineTriangle(Vector3f from, Vector3f to,
			Vector3f normal, Vector3f A, Vector3f B, Vector3f C, boolean segment) {
		dir.setSubtract(to, from);
		float r = normal.dot(dir);
		if (r <= 0) {
			return false;
		}
		CB.setSubtract(A, from);
		r = normal.dot(CB) / r;
		if ((segment && r > 1) || r < 0) {
			return false;
		}
		CB.setAddScaled(from, dir, r).subtract(A);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		float abab = AB.dot(AB);
		float acac = AC.dot(AC);
		float abac = AB.dot(AC);
		float pab = CB.dot(AB);
		float pac = CB.dot(AC);
		float denom = abac * abac - abab * acac;
		float s = (abac * pac - acac * pab) / denom;
		if (s < 0 || s > 1)
			return false;
		float t = (abac * pab - abab * pac) / denom;
		return t >= 0 && s + t <= 1;
	}

	public static boolean intersectsLineTriangle(Vector3f from, Vector3f to,
			Vector3f normal, Vector3f A, Vector3f B, Vector3f C) {
		dir.setSubtract(to, from);
		float r = normal.dot(dir);
		if (r == 0) {
			return false;
		}
		CB.setSubtract(A, from);
		r = normal.dot(CB) / r;
		CB.setAddScaled(from, dir, r).subtract(A);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		float abab = AB.dot(AB);
		float acac = AC.dot(AC);
		float abac = AB.dot(AC);
		float pab = CB.dot(AB);
		float pac = CB.dot(AC);
		float denom = abac * abac - abab * acac;
		float s = (abac * pac - acac * pab) / denom;
		if (s < 0 || s > 1)
			return false;
		float t = (abac * pab - abab * pac) / denom;
		return t >= 0 && s + t <= 1;
	}

	public static boolean intersectsLineTriangleGJK(Vector3f from,
			Vector3f dir, Vector3f normal, Vector3f A, Vector3f B, Vector3f C) {
		float r = normal.dot(dir);
		if (r == 0) {
			return false;
		}
		CB.setSubtract(A, from);
		r = normal.dot(CB) / r;
		if (r <= 0)
			return false;
		CB.setAddScaled(from, dir, r).subtract(A);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		float abab = AB.dot(AB);
		float acac = AC.dot(AC);
		float abac = AB.dot(AC);
		float pab = CB.dot(AB);
		float pac = CB.dot(AC);
		float denom = abac * abac - abab * acac;
		float s = (abac * pac - acac * pab) / denom;
		if (s < 0 || s > 1)
			return false;
		float t = (abac * pab - abab * pac) / denom;
		return t >= 0 && s + t <= 1;
	}

	public static float planeBl(Vector3f from, Vector3f to, Vector3f normal,
			Vector3f A) {
		dir.setSubtract(to, from);
		float r = normal.dot(dir);
		if (r == 0) {
			return Float.POSITIVE_INFINITY;
		}
		CB.setSubtract(A, from);
		return normal.dot(CB) / r;
	}
}
