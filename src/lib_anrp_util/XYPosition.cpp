#include "XYPosition.h"

Heading XYLine::AngleDiff(XYLine o) {
	return AngleDiff(o.first.AbsoluteBearing(o.second));
}

Heading XYLine::AngleDiff(Heading ih) {
	Heading b1 = first.AbsoluteBearing(second);

	Heading h = b1 - ih;

	if(h.GetHeading() > 180) h = h - Heading(180);
	if(h.GetHeading() > 90) h = Heading(90) - (h - Heading(90));

	return h;
}

XYPosition XYLine::CenterPoint() {
	return XYPosition((first.X() + second.X())/2.0, (first.Y() + second.Y())/2.0);
}

double XYLine::CenterDistance(XYLine o) {
	return CenterPoint().Distance(o.CenterPoint());
}

double XYLine::MinLineDistance(XYLine o) {
	for(double d = -200; d < 200; d += 1.0) {
		
	}
}

XYPosition XYPosition::ClosestPointOnLine(XYLine l) {
	XYPosition p1 = l.first, p2 = l.second;
	double xDelta = p2.X() - p1.X();
	double yDelta = p2.Y() - p1.Y();

	if ((xDelta == 0) && (yDelta == 0)) {
		throw std::runtime_error("p1 and p2 cannot be the same point");
	}

	double u = ((X() - p1.X()) * xDelta + (Y() - p1.Y()) * yDelta) / (xDelta * xDelta + yDelta * yDelta);

	XYPosition closestPoint;
	if (u < 0) {
		closestPoint = p1;
	} else if (u > 1) {
		closestPoint = p2;
	} else {
		closestPoint = XYPosition(p1.X() + u * xDelta, p1.Y() + u * yDelta);
	}

	return closestPoint;
}

double XYPosition::MinDistanceToLine(XYLine l) {
       	return ClosestPointOnLine(l).Distance(*this);
}

