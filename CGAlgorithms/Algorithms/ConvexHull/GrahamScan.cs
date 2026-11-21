using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class GrahamScan : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            // 1. edge cases
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

            // handle small sets of points (<= 3 points)
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

            // 2. step 1: find extreme vertex v and push to stack l
            // choose the point with minimum y coordinate (if tied, minimum x coordinate)
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

            // stack / list representing current convex hull
            List<Point> l = new List<Point>();
            l.Add(v);

            // collect all remaining points distinct from v
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

            // 3. angular sorting around v
            // sort remaining points in counter-clockwise order by angle with v.
            // if two points have the same angle (collinear with v), sort by distance ascending.
            remainingPoints.Sort(new RadialAngleComparer(v));

            // for collinear points with the exact same angle from v, keep only the farthest point
            List<Point> filteredPoints = new List<Point>();
            for (int j = 0; j < remainingPoints.Count; j++)
            {
                while (j < remainingPoints.Count - 1 && HelperMethods.CheckTurn(new Line(v, remainingPoints[j]), remainingPoints[j + 1]) == Enums.TurnType.Colinear)
                {
                    j++;
                }
                filteredPoints.Add(remainingPoints[j]);
            }

            // if only 1 distinct direction exists, all points are collinear
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

            // 4. push first sorted point to stack l
            l.Add(filteredPoints[0]);

            // prepare points to be explored (starting from index 1 of filtered points)
            List<Point> pointsToExplore = new List<Point>();
            for (int j = 1; j < filteredPoints.Count; j++)
            {
                pointsToExplore.Add(filteredPoints[j]);
            }

            // 5. advance phase: while points remain, check left turn (push & advance i vs pop)
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

                Point p = l[l.Count - 1];          // top(l)
                Point p_minus = l[l.Count - 2];    // previous(top(l))

                // test turn of sequence (p_minus -> p -> pi) using helpermethods.checkturn
                Enums.TurnType turn = HelperMethods.CheckTurn(new Line(p_minus, p), pi);

                if (turn == Enums.TurnType.Left)
                {
                    // strict left turn: accept pi and advance to next point
                    l.Add(pi);
                    i++;
                }
                else
                {
                    // right turn or collinear: pop p and re-evaluate pi
                    l.RemoveAt(l.Count - 1);
                }
            }

            // 6. assign result to outpoints
            for (int j = 0; j < l.Count; j++)
            {
                outPoints.Add(l[j]);
            }
        }

        /// <summary>
        /// comparer for angular sorting around vertex v.
        /// </summary>
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

                // if angles are identical (collinear with v), sort by distance ascending
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