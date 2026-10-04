/**
 * The on-screen keyboard: which fields bring it up, and when the visible strip is short by one.
 */
import assert from "node:assert/strict";
import test from "node:test";

import { KB_MIN, keyboardUp, typesText } from "../src/lib/keyboard.js";

const input = (type, more = {}) => ({ tagName: "INPUT", type, ...more });

test("text-like inputs bring the keyboard up", () => {
  for (const type of ["text", "search", "url", "email", "password", "tel", "number", "TEXT"]) {
    assert.equal(typesText(input(type)), true, type);
  }
  assert.equal(typesText({ tagName: "INPUT" }), true, "no type is text");
  assert.equal(typesText({ tagName: "textarea" }), true);
  assert.equal(typesText({ tagName: "DIV", isContentEditable: true }), true);
});

test("controls that take no typing leave it down", () => {
  for (const type of ["range", "checkbox", "radio", "button", "submit", "color", "file"]) {
    assert.equal(typesText(input(type)), false, type);
  }
  assert.equal(typesText({ tagName: "SELECT" }), false);
  assert.equal(typesText({ tagName: "BUTTON" }), false);
  assert.equal(typesText({ tagName: "DIV" }), false);
  assert.equal(typesText(null), false);
  assert.equal(typesText(undefined), false);
});

test("a read-only or disabled field leaves it down", () => {
  // The message field is read-only while it asks which agent to use, and that question is a drawer.
  assert.equal(typesText(input("text", { readOnly: true })), false);
  assert.equal(typesText(input("text", { disabled: true })), false);
  assert.equal(typesText({ tagName: "TEXTAREA", readOnly: true }), false);
});

test("the strip counts as keyboard-short from KB_MIN", () => {
  // An iPhone 16 Pro's 852pt page with its keyboard up leaves about 480pt.
  assert.equal(keyboardUp(852, 480), true);
  assert.equal(keyboardUp(852, 852), false);
  assert.equal(keyboardUp(852, 852 - KB_MIN), true);
  assert.equal(keyboardUp(852, 852 - KB_MIN + 1), false);
  // An iPad's shortcut bar over a hardware keyboard is no keyboard.
  assert.equal(keyboardUp(1180, 1180 - 55), false);
});

test("pinch zoom shrinks the strip without a keyboard", () => {
  assert.equal(keyboardUp(852, 426, 2), false);
  assert.equal(keyboardUp(852, 240, 2), true);
});
