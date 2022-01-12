//Code heavily taken from Example Code and Explanations by Paul Bourke at http://paulbourke.net/geometry/pointlineplane/
//
//Function to test for intersections between line segments:
function intersectionPt(x1, x2, x3, x4, y1, y2, y3, y4) {

    let uA, uB;
    let den, numA, numB;

    den = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    numA = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    numB = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);

    //Coincident? 
    if (abs(numA) == 0 && abs(numB) == 0 && abs(den) == 0) {
        intx = (x1 + x2) / 2;
        inty = (y1 + y2) / 2;
        return (true);
    }

    //Parallel? - No intersection
    if (abs(den) == 0) {
        intx = 0;
        inty = 0;
        return (false);
    }

    //Intersection?
    uA = numA / den;
    uB = numB / den;

    //If both lie w/in the range of 0 to 1 then the intersection point is within both line segments.
    if (uA < 0 || uA > 1 || uB < 0 || uB > 1) {
        intx = 0;
        inty = 0;
        return (false);
    }
    intx = x1 + uA * (x2 - x1);
    inty = y1 + uA * (y2 - y1);
    return (true);
}