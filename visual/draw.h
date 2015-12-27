#ifndef __TK_DRAW_H__
#define __TK_DRAW_H__

#include <armadillo>

/** Draw a rectangle
 *  @param I the image to draw a rectangle in (gray)
 *  @param v the grayscale intensity
 *  @param x the top-left corner x of the rectangle
 *  @param y the top-left corner y of the rectangle
 *  @param width the width of the rectangle
 *  @param height the height of the rectangle
 */
void draw_rect(arma::mat &I, double v, int x, int y, int width, int height);

/** Draw a rectangle
 *  @param I the image to draw a rectangle in (rgb)
 *  @param v the rgb intensity vector
 *  @param x the top-left corner x of the rectangle
 *  @param y the top-left corner y of the rectangle
 *  @param width the width of the rectangle
 *  @param height the height of the rectangle
 */
void draw_rect(arma::cube &I, const arma::vec &v, int x, int y, int width, int height);

/** Draw a line
 *  @param I the image to draw a line in (gray)
 *  @param v the grayscale intensity
 *  @param x1 the first coord x
 *  @param y1 the first coord y
 *  @param x2 the second coord x
 *  @param y2 the second coord y
 */
void draw_line(arma::mat &I, double v, int x1, int y1, int x2, int y2);

/** Draw a line
 *  @param I the image to draw a line in (rgb)
 *  @param v the rgb intensity vector
 *  @param x1 the first coord x
 *  @param y1 the first coord y
 *  @param x2 the second coord x
 *  @param y2 the second coord y
 */
void draw_line(arma::cube &I, const arma::vec &v, int x1, int y1, int x2, int y2);

/** Draw a circle
 *  @param I the image to draw a circle in (gray)
 *  @param v the grayscale intensity
 *  @param x the center coord x
 *  @param y the center coord y
 *  @param radius the radius of the circle
 */
void draw_circle(arma::mat &I, double v, int x, int y, double radius);

/** Draw a circle
 *  @param I the image to draw a circle in (gray)
 *  @param v the rgb intensity
 *  @param x the center coord x
 *  @param y the center coord y
 *  @param radius the radius of the circle
 */
void draw_circle(arma::cube &I, const arma::vec &v, int x, int y, double radius);

#endif
