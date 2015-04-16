package com.brocorporation.gameengine.elements.collision;

import java.util.Stack;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.bodies.DynamicBody;
import com.brocorporation.gameengine.elements.bodies.StaticBody;

public class ElasticContactSolver {

	private final static Stack<ElasticContact> useStack = new Stack<ElasticContact>();
	private final static Stack<ElasticContact> unusedStack = new Stack<ElasticContact>();

	public static void addContact(final StaticBody stcBody,
			final DynamicBody dynBody, final Contact contact) {
		try {
			final ElasticContact c;
			if (unusedStack.isEmpty()) {
				c = new ElasticContact(stcBody, dynBody, contact);
			} else {
				c = unusedStack.pop();
				c.reset(stcBody, dynBody, contact);
			}
			useStack.push(c);
		} catch (Exception e) {
		}
	}

	public static void run(final IUpdateInfo uInfo) {
		while (!useStack.isEmpty()) {
			final ElasticContact c;
			if ((c = useStack.pop()) != null) {
				c.solve(uInfo);
				c.reset();
				unusedStack.push(c);
			}
		}
	}

	public static void releaseAll() {
		while (!useStack.isEmpty()) {
			final ElasticContact c;
			if ((c = useStack.pop()) != null) {
				c.reset();
				unusedStack.push(c);
			}
		}
	}
}
