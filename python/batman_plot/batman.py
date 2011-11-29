#! /usr/bin/env python
# usage: python batman.py 
# 
# plot batman symbol
# equation from: http://www.reddit.com/r/pics/comments/j2qjc/do_you_like_batman_do_you_like_math_my_math/
# 
# Jordan Brindza - July, 2011
#

import numpy as np
import matplotlib.pyplot as mpl


# for reference
# p1 = ((x/7.0)**2.0 * np.sqrt((np.abs(np.abs(x)-3.0))/(np.abs(3.0)-3.0))) + (((y/3.0)**2.0 * np.sqrt(np.abs(y + (3.0*np.sqrt(33.0)/7.0))/(y + (3.0*np.sqrt(33.0)/7.0)))) - 1.0);
# p2 = (np.abs(x/2.0) - (((3.0*np.sqrt(33.0) - 7.0)/112.0) * x**2.0) - 3.0 + np.sqrt(1.0 - (np.abs(np.abs(x) - 2.0) - 1.0)**2.0) - y);
# p3 = (9.0 * np.sqrt(np.abs((np.abs(x) - 1.0)*(np.abs(x) - 0.75))/((1.0 - np.abs(x))*(np.abs(x) - 0.75))) - 8.0*np.abs(x) - y);
# p4 = (3.0*np.abs(x) + 0.75*np.sqrt(np.abs((np.abs(x) - 0.75)*(np.abs(x) - 0.5))/((0.75 - np.abs(x))*(np.abs(x) - 0.5))) - y);
# p5 = (2.25 * np.sqrt(np.abs((x - 0.5)*(x + 0.5))/((0.5 - x)*(0.5 + x))) - y);
# p6 = ((6.0*np.sqrt(10.0)/7.0) + (1.5 - 0.5*np.abs(x)*np.sqrt(np.abs(np.abs(x) - 1.0)/(np.abs(x) - 1.0)) - (6.0*np.sqrt(10.0)/14.0)*np.sqrt(4.0 - (np.abs(x) - 1.0)**2.0) - y));
# p1 * p2 * p3 * p4 * p5 * p6 = 0


def solve_p1(x):
  y = 3.0*(np.sqrt(1.0 - (x/7.0)**2.0))
  y[x <= 3] = np.nan;
  yp = y.copy();
  yn = -y.copy();
  yn[yn < -(3.0*np.sqrt(33)/7.0)] = np.nan;
  return (yp, yn);

def solve_p2(x):
  y = (np.abs(x/2.0) - (((3.0*np.sqrt(33.0) - 7.0)/112.0) * x**2.0) - 3.0 + np.sqrt(1.0 - (np.abs(np.abs(x) - 2.0) - 1.0)**2.0));
  return y;

def solve_p3(x):
  y = (9.0 * np.sqrt(np.abs((np.abs(x) - 1.0)*(np.abs(x) - 0.75))/((1.0 - np.abs(x))*(np.abs(x) - 0.75))) - 8.0*np.abs(x));
  return y;

def solve_p4(x):
  y = (3.0*np.abs(x) + 0.75*np.sqrt(np.abs((np.abs(x) - 0.75)*(np.abs(x) - 0.5))/((0.75 - np.abs(x))*(np.abs(x) - 0.5))));
  return y;

def solve_p5(x):
  y = (2.25 * np.sqrt(np.abs((x - 0.5)*(x + 0.5))/((0.5 - x)*(0.5 + x))));
  return y;

def solve_p6(x):
  y = ((6.0*np.sqrt(10.0)/7.0) + (1.5 - 0.5*np.abs(x)*np.sqrt(np.abs(np.abs(x) - 1.0)/(np.abs(x) - 1.0)) - (6.0*np.sqrt(10.0)/14.0)*np.sqrt(4.0 - (np.abs(x) - 1.0)**2.0)));
  return y;


if __name__=='__main__':
  xmin = -8.0;
  xmax = 8.0;
  xres = 100000;

  x = np.arange(xmin, xmax, (xmax-xmin)/xres);
  

  (y1p, y1n) = solve_p1(x);
  y2 = solve_p2(x);
  y3 = solve_p3(x);
  y4 = solve_p4(x);
  y5 = solve_p5(x);
  y6 = solve_p6(x);

  
  mpl.plot(x, y1p, 'k');
  mpl.plot(-x, y1p, 'k');
  mpl.plot(x, y1n, 'k');
  mpl.plot(-x, y1n, 'k');
  mpl.plot(x, y2, 'k');
  mpl.plot(-x, y2, 'k');
  mpl.plot(x, y3, 'k');
  mpl.plot(x, y4, 'k');
  mpl.plot(x, y5, 'k');
  mpl.plot(x, y6, 'k');
  mpl.show();

