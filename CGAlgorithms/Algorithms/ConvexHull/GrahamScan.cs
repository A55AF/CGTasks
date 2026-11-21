using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class GrahamScan : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            // base cases
            if (points == null || points.Count == 0)
            {
                return;
            }

            if (outPoints == null)
            {
                outPoints = new List<Point>();
            }
            else
            {
                outPoints.Clear();
            }

            // trivial cases (3 or fewer points)
            if (points.Count <= 3)
            {
                for (int j = 0; j < points.Count; j++)
                {
                    bool isDuplicate = false;
                    for (int k = 0; k < outPoints.Count; k++)
                    {
                        if (points[j].Equals(outPoints[k]))
                        {
                            isDuplicate = true;
                            break;
                        }
                    }

                    if (!isDuplicate)
                    {
                        outPoints.Add(points[j]);
                    }
                }
                return;
            }

            // find anchor point with min y (min x on tie)
            Point v = points[0];
            for (int j = 1; j < points.Count; j++)
            {
                if (points[j].Y < v.Y)
                {
                    v = points[j];
                }
                else if (points[j].Y == v.Y && points[j].X < v.X)
                {
                    v = points[j];
                }
            }

            // stack to store hull vertices
            List<Point> l = new List<Point>();
            l.Add(v);

            // get remaining points excluding pivot v
            List<Point> remainingPoints = new List<Point>();
            for (int j = 0; j < points.Count; j++)
            {
                if (!points[j].Equals(v))
                {
                    remainingPoints.Add(points[j]);
                }
            }

            if (remainingPoints.Count == 0)
            {
                outPoints.Add(v);
                return;
            }

            // sort by polar angle counter-clockwise around v
            remainingPoints.Sort(new RadialAngleComparer(v));

            // for collinear points on the same angle from v, keep only the farthest
            List<Point> filteredPoints = new List<Point>();
            for (int j = 0; j < remainingPoints.Count; j++)
            {
                while (j < remainingPoints.Count - 1 && HelperMethods.CheckTurn(new Line(v, remainingPoints[j]), remainingPoints[j + 1]) == Enums.TurnType.Colinear)
                {
                    j++;
                }
                filteredPoints.Add(remainingPoints[j]);
            }

            // all points lie on a straight line
            if (filteredPoints.Count < 2)
            {
                for (int j = 0; j < filteredPoints.Count; j++)
                {
                    l.Add(filteredPoints[j]);
                }
                for (int j = 0; j < l.Count; j++)
                {
                    outPoints.Add(l[j]);
                }
                return;
            }

            // push first point to stack
            l.Add(filteredPoints[0]);

            // remaining points to scan
            List<Point> pointsToExplore = new List<Point>();
            for (int j = 1; j < filteredPoints.Count; j++)
            {
                pointsToExplore.Add(filteredPoints[j]);
            }

            // advance through points and maintain left turns
            int i = 0;
            while (i < pointsToExplore.Count)
            {
                Point pi = pointsToExplore[i];

                if (l.Count < 2)
                {
                    l.Add(pi);
                    i++;
                    continue;
                }

                Point p = l[l.Count - 1];          // top
                Point p_minus = l[l.Count - 2];    // second from top

                Enums.TurnType turn = HelperMethods.CheckTurn(new Line(p_minus, p), pi);

                if (turn == Enums.TurnType.Left)
                {
                    // keep point and move forward
                    l.Add(pi);
                    i++;
                }
                else
                {
                    // non-left turn, pop top of stack and retry pi
                    l.RemoveAt(l.Count - 1);
                }
            }

            for (int j = 0; j < l.Count; j++)
            {
                outPoints.Add(l[j]);
            }
        }

        // custom comparer for polar angle sorting
        private class RadialAngleComparer : IComparer<Point>
        {
            private Point v;

            public RadialAngleComparer(Point v)
            {
                this.v = v;
            }

            public int Compare(Point p1, Point p2)
            {
                double angle1 = Math.Atan2(p1.Y - v.Y, p1.X - v.X);
                double angle2 = Math.Atan2(p2.Y - v.Y, p2.X - v.X);

                if (Math.Abs(angle1 - angle2) > Constants.Epsilon)
                {
                    return angle1.CompareTo(angle2);
                }

                // break ties with distance from v
                double dist1 = v.Vector(p1).Magnitude();
                double dist2 = v.Vector(p2).Magnitude();

                return dist1.CompareTo(dist2);
            }
        }

        public override string ToString()
        {
            return "Convex Hull - Graham Scan";
        }
    }
}