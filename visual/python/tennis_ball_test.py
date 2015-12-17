from SimpleCV import *
import tennis_ball as ball


def unitTest(actual, expected, name):
    if actual == expected:
        print "Test '%s' passed" % name
    else:
        print "Test '%s' failed" % name

def ballPresent():
    img = Image("assets/test/present1_ball.jpg")
    unitTest(ball.is_ball_middle(img), False, "Present 1")

    img = Image("assets/test/present2_ball.jpg")
    unitTest(ball.is_ball_middle(img), True, "Present 2")

    img = Image("assets/test/present3_ball.jpg")
    unitTest(ball.is_ball_middle(img), True, "Present 3")

def ballMissing():
    img = Image("assets/test/missing1.jpg")
    unitTest(ball.is_ball_middle(img), False, "Missing 1")

    img = Image("assets/test/missing2_ball.jpg")
    unitTest(ball.is_ball_middle(img), False, "Missing 2")

    img = Image("assets/test/missing3.jpg")
    unitTest(ball.is_ball_middle(img), False, "Missing 3")

ballMissing()
ballPresent()
