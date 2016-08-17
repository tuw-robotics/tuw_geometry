#!/usr/bin/env python
import tuw_geometry

if __name__ == '__main__':
    p = tuw_geometry.PointX()
    p.set("servus")
    print p.greet()
    
    pnt0 = tuw_geometry.Point2D()
    pnt1 = tuw_geometry.Point2D(3,2)
    pnt2 = tuw_geometry.Point2D(3,2,4)
    print (pnt1)