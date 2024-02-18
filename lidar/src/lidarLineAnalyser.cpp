#include <lidarLineAnalyser.h>

using namespace cv;
using namespace std;

LineAnalyser::LineAnalyser(vector<Vec<Point, 2>> lines)
{
    //Convert to SLines
    for (auto i = lines.begin(); i != lines.end(); i++)
    {
        anaLines.push_back(SLine(*i));
    }

    // calcOrientation();
    // calcPosition();
    // calcMap();
}

double LineAnalyser::getOrientation()
{
    if (!orientationCalced) calcOrientation();
    return orientation;
}

Point LineAnalyser::getTilePosition()
{
    if (!positionCalced) calcPosition();
    return position;
}

array<array<Vec<bool, 4>, TILE_READ_AMOUNT>, TILE_READ_AMOUNT> LineAnalyser::getMap()
{
    if (!mapCalced) calcMap();
    return relMap;
}

void LineAnalyser::calcOrientation()
{
    //Find longest line, which we assume is a wall

    size_t longestLineIndex = 0;

    for (size_t i = 0; i < anaLines.size(); i++)
    {
        if (anaLines[i].length > anaLines[longestLineIndex].length)
        {
            longestLineIndex = i;
        }
    }

    //Get the angle of the longest line, other walls should have similar orientations or perpendicular ones.
    double baseAngle = anaLines[longestLineIndex].orientation; //Orientation of the longest line
    double perpAngle = pi2Mod(baseAngle - M_PI/2);             //Perpendiculat line to the longest line

    double baseLineAngDiffSum = 0; //Sum of orientation diff from baseline - from lines that have ~same orientation as baseangle (likely walls)
    int baseLineAmount = 0; //Amount of these lines
    double perpLineAngDiffSum = 0; //Sum of orientation diff from perpendicular to baseline - from lines that have ~same orientation as the perpendicular to baseangle (likely walls)
    int perpLineAmount = 0; //Amount of these lines

    const double wallLengthThreshold = 200; //Minimum length of a line for it to be seen as a wall is 200 mm
    const double angDiffThreshold = M_PI/6; //Minimum angle for a line to be seen as similar to base/perpAngle


    //Add line orientations for likely walls
    for (auto i = anaLines.begin(); i != anaLines.end(); i++)
    {
        if(i->length < wallLengthThreshold) continue;

        double baseDiff = piMod(i->orientation - baseAngle);
        double perpDiff = piMod(i->orientation - perpAngle);

        if (abs(baseDiff) < angDiffThreshold)
        {
            wallLines.push_back(*i);
            baseLineAngDiffSum += baseDiff;
            baseLineAmount++;
            cout << i->orientation*180/M_PI << "⁰ - added as base" << endl;
        }
        else if (abs(perpDiff) < angDiffThreshold)
        {
            wallLines.push_back(*i);
            perpLineAngDiffSum += perpDiff;
            perpLineAmount++;
            cout << i->orientation*180/M_PI << "⁰ - added as perp" << endl;
        }
    }

    //Form orientation averages
    double baseAvg = baseAngle + baseLineAngDiffSum/baseLineAmount;
    double perpAvg = perpAngle + perpLineAngDiffSum/perpLineAmount;

    if (perpLineAmount > 0)
	{
        orientation = (baseAvg + pi2Mod(perpAvg + M_PI/2))/2;
	}
    else
	{
        orientation = baseAvg;
	}
	orientation = orientation - round(baseAngle*2*M_1_PI)*M_PI_2; //It is now an angle against the x-axis???
    cout << "Orientation: " << orientation*180/M_PI << "⁰; basetot: " << baseAvg*180/M_PI << "⁰; perptot: " << perpAvg*180/M_PI << "⁰, longest: " << anaLines[longestLineIndex].orientation*180/M_PI << endl;
    cout << "or" << orientation << endl;
    orientationCalced = true;
}

void LineAnalyser::calcPosition()
{
    if (!orientationCalced) calcOrientation();

    double xPosTot = 0;
    double yPosTot = 0;
    int xPosAmt = 0;
    int yPosAmt = 0;

    // vector<Vec<Point, 2>> xWallLines;
    // vector<Vec<Point, 2>> yWallLines;

    for (auto i = wallLines.begin(); i != wallLines.end(); i++)
    {
        Point midPoint = transformPoint(i->closestToOrigin, ORIGIN, orientation);
        // circle(showLines, midPoint+ORIGIN, 10, Scalar(255, 255, 100), 10);
        // cout << "MIDPOS: " << midPoint.x << ", " << midPoint.y << " ,, grad " << i->orientation * 180/M_PI << "->" << abs(piMod(orientation - i->orientation)) * 180/M_PI << endl;

        double angleDiffAbs = abs(piMod(orientation - i->orientation));
        if (angleDiffAbs >= M_PI_4)
        {
            //LEFTRIGHT
            // xWallLines.push_back({i->startPoint, i->endPoint});
			double currXPosAvg = 0;
			if (xPosAmt != 0)
				currXPosAvg = xPosTot/xPosAmt;

			int xPosPart = tileMod(midPoint.x, currXPosAvg);
            xPosTot += xPosPart;
            xPosAmt++;

            if (midPoint.x > 0)
            {
                //RIGHT SIDE, xpos
                cout << "._.xp" << xPosPart << endl;
            }
            else
            {
                //LEFT SIDE, xneg
                cout << "._.xn" << xPosPart << endl;
            }
        }
        else
        {
            //UPDOWN
            // yWallLines.push_back({i->startPoint, i->endPoint});
			double currYPosAvg = 0;
			if (yPosAmt != 0)
				currYPosAvg = yPosTot/yPosAmt;
			
			int yPosPart = tileMod(midPoint.y, currYPosAvg);
            yPosTot += yPosPart;
            yPosAmt++;

            if (midPoint.y > 0)
            {
                //BACK SIDE, ypos
                cout << "._.yp" << yPosPart << endl;
            }
            else
            {
                //FRONT SIDE, yneg
                cout << "._.yn" << yPosPart << endl;
            }
        }
    }

    double xPosAvg = 0;
    if (xPosAmt > 0) 
	{
		xPosAvg = xPosTot/xPosAmt;
		if (xPosAvg < 0) xPosAvg+=300;
	}
    else cout << "ERROR NO XPOS CONTRIB" << endl;

    double yPosAvg = 0;
    if (yPosAmt > 0)
	{
		yPosAvg = yPosTot/yPosAmt;
		if (yPosAvg < 0) yPosAvg+=300;
	}
    else cout << "ERROR NO YPOS CONTRIB" << endl;
    
    cout << "POS: " << xPosAvg << ", " << yPosAvg << endl;

    position = Point(xPosAvg, yPosAvg);
    positionCalced = true;
}

void LineAnalyser::calcMap()
{
    if (!orientationCalced) calcOrientation();
    if (!positionCalced) calcPosition();
    //This should maybe be changed...
    int tileEndPointsXY[TILE_READ_AMOUNT+1];
    for (size_t i = 0; i < TILE_READ_AMOUNT+1; i++)
    {
        tileEndPointsXY[i] = 300 * i - 300*(TILE_READ_AMOUNT)/2;
    }

    for (auto i = wallLines.begin(); i != wallLines.end(); i++)
    {
        //Check the closest wall endpoints to line endpoints, draw lines between

        Point startPointRebased = transformPoint(i->startPoint, ORIGIN, orientation) - position + Point(150, 150); // + OR - ??
        Point endPointRebased = transformPoint(i->endPoint, ORIGIN, orientation) - position + Point(150, 150); // + OR - ??
        // renderPoint.push_back(startPointRebased+ORIGIN);
        // renderPoint.push_back(endPointRebased+ORIGIN);

        int minDiffStartXIndex = -1;
        int minDiffStartYIndex = -1;
        int minDiffEndXIndex = -1;
        int minDiffEndYIndex = -1;
        for (size_t j = 0; j < TILE_READ_AMOUNT+1; j++)
        {
            if (abs(tileEndPointsXY[j] - startPointRebased.x) <= 150) //Can be changed to threshold points that are too far
                minDiffStartXIndex = j;
            if (abs(tileEndPointsXY[j] - startPointRebased.y) <= 150)
                minDiffStartYIndex = j;
            if (abs(tileEndPointsXY[j] - endPointRebased.x) <= 150)
                minDiffEndXIndex = j;
            if (abs(tileEndPointsXY[j] - endPointRebased.y) <= 150)
                minDiffEndYIndex = j;
        }

        if (minDiffEndXIndex == -1 || minDiffEndYIndex == -1 || minDiffStartXIndex == -1 || minDiffStartYIndex == -1)
            cout << "wall too far away" << endl;
        else if ((minDiffEndXIndex == minDiffStartXIndex) && (minDiffEndYIndex == minDiffStartYIndex))
        	cout << "too short line" << endl;
        else
        {
			if (minDiffEndXIndex == minDiffStartXIndex)
			{
				int smallerIndex = min(minDiffEndYIndex, minDiffStartYIndex);
				int largerIndex = max(minDiffEndYIndex, minDiffStartYIndex);
				bool setNext = minDiffEndXIndex > 0;
				for (int j = smallerIndex; j < largerIndex; j++)
				{
					relMap[j][minDiffEndXIndex][1] = true;
					if(setNext) relMap[j][minDiffEndXIndex-1][3] = true;
				}
			}
			else if (minDiffEndYIndex == minDiffStartYIndex)
			{
				int smallerIndex = min(minDiffEndXIndex, minDiffStartXIndex);
				int largerIndex = max(minDiffEndXIndex, minDiffStartXIndex);
				bool setNext = minDiffEndYIndex > 0;
				for (int j = smallerIndex; j < largerIndex; j++)
				{
					relMap[minDiffEndYIndex][j][0] = true;
					if(setNext) relMap[minDiffEndYIndex-1][j][2] = true;
				}
			}
			else
				cout << "line not straight" << endl; //Line was not in one column or row, but multiple
        }
    }

    mapCalced = true;
}

LineAnalyser::SLine::SLine(Vec<Point, 2> linePoints)
{
    
    if (linePoints[0].x <= linePoints[1].x)
    {
        startPoint = linePoints[0];
        endPoint = linePoints[1];
    }
    else
    {
        startPoint = linePoints[1];
        endPoint = linePoints[0];
    }
    orientation = piMod(atan2((endPoint.y - startPoint.y), (endPoint.x - startPoint.x)));
    length = norm(endPoint - startPoint);

    //From transformation: distance(P,θ,(x0,y0)) = |cos(θ)(Py-y0)-sin(θ)(Px-xθ)|
    double minDistance = cos(orientation) * (startPoint.y - ORIGIN_Y) - sin(orientation) * (startPoint.x - ORIGIN_X);
    // double startXTransformed = (startPoint.y - ORIGIN_Y) * sin(orientation) + (startPoint.x - ORIGIN_X) * cos(orientation);
    // double endXTransformed = (endPoint.y - ORIGIN_Y) * sin(orientation) + (endPoint.x - ORIGIN_X) * cos(orientation);
    // if NOT (min(startXTransformed, endXTransformed) < 0 && max(startXTransformed, endXTransformed) > 0) -> line is less probable
    closestToOrigin = Point(minDistance * cos(orientation + M_PI/2) + ORIGIN_X, minDistance * sin(orientation + M_PI/2) + ORIGIN_Y);
}