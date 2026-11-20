using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class GrahamScan : Algorithm
    {
        public override void Run(List<Point> points, List<Line> lines, List<Polygon> polygons, ref List<Point> outPoints, ref List<Line> outLines, ref List<Polygon> outPolygons)
        {
            // 1. Edge cases
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

            // Handle small sets of points (<= 3 points)
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

            // 2. Step 1: Find extreme vertex v and push to stack l
            // Choose the point with minimum Y coordinate (if tied, minimum X coordinate)
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

            // Stack / list representing current convex hull
            List<Point> l = new List<Point>();
            l.Add(v);

            // Collect all remaining points distinct from v
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

            // 3. Angular sorting around v
            // Sort remaining points in counter-clockwise order by angle with v.
            // If two points have the same angle (collinear with v), sort by distance ascending.
            remainingPoints.Sort(new RadialAngleComparer(v));

            // For collinear points with the exact same angle from v, keep only the farthest point
            List<Point> filteredPoints = new List<Point>();
            for (int j = 0; j < remainingPoints.Count; j++)
            {
                while (j < remainingPoints.Count - 1 && IsSameRay(v, remainingPoints[j], remainingPoints[j + 1]))
                {
                    j++;
                }
                filteredPoints.Add(remainingPoints[j]);
            }

            // If only 1 distinct direction exists, all points are collinear
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

            // 4. Push first sorted point to stack l
            l.Add(filteredPoints[0]);

            // Prepare points to be explored (starting from index 1 of filtered points)
            List<Point> pointsToExplore = new List<Point>();
            for (int j = 1; j < filteredPoints.Count; j++)
            {
                pointsToExplore.Add(filteredPoints[j]);
            }

            // 5. Advance phase: While points remain, check left turn (push & advance i vs pop)
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

                // Test turn of sequence (p_minus -> p -> pi) using 2D cross product
                double crossProduct = GetCrossProduct(p_minus, p, pi);

                if (crossProduct > Constants.Epsilon)
                {
                    // Strict Left Turn: accept pi and advance to next point
                    l.Add(pi);
                    i++;
                }
                else
                {
                    // Right Turn or Collinear: pop p and re-evaluate pi
                    l.RemoveAt(l.Count - 1);
                }
            }

            // 6. Assign result to outPoints
            for (int j = 0; j < l.Count; j++)
            {
                outPoints.Add(l[j]);
            }
        }

        /// <summary>
        /// Calculates the 2D cross product of vectors (p - p_minus) and (pi - p_minus).
        /// Value > 0: Strict Left Turn (Counter-Clockwise)
        /// Value < 0: Right Turn (Clockwise)
        /// Value == 0: Collinear
        /// </summary>
        private double GetCrossProduct(Point p_minus, Point p, Point pi)
        {
            return (p.X - p_minus.X) * (pi.Y - p_minus.Y) - (p.Y - p_minus.Y) * (pi.X - p_minus.X);
        }

        /// <summary>
        /// Checks if two points p1 and p2 lie on the same ray from origin point v.
        /// </summary>
        private bool IsSameRay(Point v, Point p1, Point p2)
        {
            double crossProduct = GetCrossProduct(v, p1, p2);
            return Math.Abs(crossProduct) <= Constants.Epsilon;
        }

        /// <summary>
        /// Comparer for angular sorting around vertex v.
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

                // If angles are identical (collinear with v), sort by squared distance ascending
                double dist1 = (p1.X - v.X) * (p1.X - v.X) + (p1.Y - v.Y) * (p1.Y - v.Y);
                double dist2 = (p2.X - v.X) * (p2.X - v.X) + (p2.Y - v.Y) * (p2.Y - v.Y);

                return dist1.CompareTo(dist2);
            }
        }

        public override string ToString()
        {
            return "Convex Hull - Graham Scan";
        }
    }
}
