using CGUtilities;
using System;
using System.Collections.Generic;

namespace CGAlgorithms.Algorithms.ConvexHull
{
    public class JarvisMarch : Algorithm
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
                // Eliminate duplicates (if any) or add points directly to outPoints
                for (int i = 0; i < points.Count; i++)
                {
                    bool isDuplicate = false;
                    for (int j = 0; j < outPoints.Count; j++)
                    {
                        if (points[i].Equals(outPoints[j]))
                        {
                            isDuplicate = true;
                            break;
                        }
                    }

                    if (!isDuplicate)
                    {
                        outPoints.Add(points[i]);
                    }
                }
                return;
            }

            // 2. Step 1: Lexicographically smallest point
            // Find point with minimum X coordinate (tie-break with minimum Y coordinate)
            Point p0 = points[0];
            for (int i = 1; i < points.Count; i++)
            {
                if (points[i].X < p0.X)
                {
                    p0 = points[i];
                }
                else if (points[i].X == p0.X && points[i].Y < p0.Y)
                {
                    p0 = points[i];
                }
            }

            outPoints.Add(p0);

            // 3. Step 2: Wrapping loop (detecting angularly rightmost point)
            Point v = p0;

            while (true)
            {
                // Pick an initial candidate point pj from points that is not equal to v
                Point pj = null;
                for (int i = 0; i < points.Count; i++)
                {
                    if (!points[i].Equals(v))
                    {
                        pj = points[i];
                        break;
                    }
                }

                // If all points are identical to v, no other candidate exists
                if (pj == null)
                {
                    break;
                }

                // Iterate over all points pk in points to find the most clockwise / rightmost turn
                for (int i = 0; i < points.Count; i++)
                {
                    Point pk = points[i];

                    // Skip evaluating the current hull vertex itself
                    if (pk.Equals(v))
                    {
                        continue;
                    }

                    // Evaluate orientation of triplet (v, pj, pk) using 2D cross product:
                    // CrossProduct = (pj.X - v.X) * (pk.Y - v.Y) - (pj.Y - v.Y) * (pk.X - v.X)
                    double crossProduct = GetCrossProduct(v, pj, pk);

                    if (crossProduct < -Constants.Epsilon)
                    {
                        // pk is angularly more to the right of ray (v -> pj)
                        pj = pk;
                    }
                    else if (Math.Abs(crossProduct) <= Constants.Epsilon)
                    {
                        // 4. Collinear tie-breaking:
                        // If v, pj, and pk are collinear, choose the point farthest from v
                        if (GetDistanceSquared(v, pk) > GetDistanceSquared(v, pj))
                        {
                            pj = pk;
                        }
                    }
                }

                // 5. Termination check:
                // If the detected rightmost point equals the starting point, the hull has wrapped around
                if (pj.Equals(outPoints[0]))
                {
                    break;
                }

                // Add the next hull vertex and advance
                outPoints.Add(pj);
                v = pj;
            }
        }

        /// <summary>
        /// Calculates the 2D cross product of vectors (a -> b) and (a -> c).
        /// Value < 0: Right turn (clockwise)
        /// Value > 0: Left turn (counter-clockwise)
        /// Value == 0: Collinear
        /// </summary>
        private double GetCrossProduct(Point a, Point b, Point c)
        {
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        }

        /// <summary>
        /// Calculates the squared Euclidean distance between two points.
        /// </summary>
        private double GetDistanceSquared(Point a, Point b)
        {
            double dx = a.X - b.X;
            double dy = a.Y - b.Y;
            return dx * dx + dy * dy;
        }

        public override string ToString()
        {
            return "Convex Hull - Jarvis March";
        }
    }
}
