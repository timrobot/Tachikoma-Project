from SimpleCV import *
import basket


def unitTest(actual, expected, name):
    if actual == expected:
        print "Test '%s' passed" % name
    else:
        print "Test '%s' failed" % name

def basketPresent():
    img = Image("assets/test/present1.jpg")
    unitTest(basket.is_basket_middle(img), True, "Present 1")

    img = Image("assets/test/present2.jpg")
    unitTest(basket.is_basket_middle(img), True, "Present 2")

    img = Image("assets/test/present3.jpg")
    unitTest(basket.is_basket_middle(img), True, "Present 3")

def basketMissing():
    img = Image("assets/test/missing1.jpg")
    unitTest(basket.is_basket_middle(img), False, "Missing 1")

    img = Image("assets/test/missing2.jpg")
    unitTest(basket.is_basket_middle(img), False, "Missing 2")

    img = Image("assets/test/missing3.jpg")
    unitTest(basket.is_basket_middle(img), False, "Missing 3")

basketPresent()
basketMissing()
